# PhotoPi/photopack/main_system/retry_utils.py
"""
Retry utilities for robust network operations.
Provides decorators and utilities for handling transient failures.
"""

import time
import random
import logging
from functools import wraps
from typing import Callable, Type, Tuple, Any, Optional, Union
import paramiko
import socket

logger = logging.getLogger(__name__)


class RetryableError(Exception):
    """Base exception for errors that should trigger a retry."""
    pass


class NonRetryableError(Exception):
    """Exception for errors that should not trigger a retry."""
    pass


# Define which exceptions are retryable
RETRYABLE_EXCEPTIONS = (
    paramiko.SSHException,
    socket.timeout,
    socket.error,
    ConnectionResetError,
    ConnectionRefusedError,
    BrokenPipeError,
    RetryableError,
)

NON_RETRYABLE_EXCEPTIONS = (
    paramiko.AuthenticationException,
    paramiko.BadAuthenticationType,
    NonRetryableError,
    KeyboardInterrupt,
    SystemExit,
)


def exponential_backoff(attempt: int, base_delay: float = 1.0, max_delay: float = 60.0, jitter: bool = True) -> float:
    """
    Calculate exponential backoff delay with optional jitter.
    
    Args:
        attempt: Current attempt number (starting from 1)
        base_delay: Base delay in seconds
        max_delay: Maximum delay in seconds
        jitter: Whether to add random jitter
        
    Returns:
        Delay in seconds
    """
    delay = min(base_delay * (2 ** (attempt - 1)), max_delay)
    
    if jitter:
        # Add ±25% jitter to prevent thundering herd
        jitter_range = delay * 0.25
        delay += random.uniform(-jitter_range, jitter_range)
    
    return max(0, delay)


def retry(
    max_attempts: int = 3,
    base_delay: float = 1.0,
    max_delay: float = 60.0,
    backoff_multiplier: float = 2.0,
    jitter: bool = True,
    retryable_exceptions: Tuple[Type[Exception], ...] = RETRYABLE_EXCEPTIONS,
    non_retryable_exceptions: Tuple[Type[Exception], ...] = NON_RETRYABLE_EXCEPTIONS,
    on_retry: Optional[Callable[[int, Exception], None]] = None,
    on_failure: Optional[Callable[[Exception], None]] = None
):
    """
    Decorator for retrying function calls with exponential backoff.
    
    Args:
        max_attempts: Maximum number of attempts
        base_delay: Initial delay between retries in seconds
        max_delay: Maximum delay between retries in seconds
        backoff_multiplier: Multiplier for exponential backoff
        jitter: Whether to add random jitter to delays
        retryable_exceptions: Exceptions that should trigger a retry
        non_retryable_exceptions: Exceptions that should not trigger a retry
        on_retry: Callback function called on each retry (attempt, exception)
        on_failure: Callback function called when all retries are exhausted
        
    Returns:
        Decorated function
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            last_exception = None
            
            for attempt in range(1, max_attempts + 1):
                try:
                    result = func(*args, **kwargs)
                    if attempt > 1:
                        logger.info(f"{func.__name__} succeeded on attempt {attempt}")
                    return result
                    
                except non_retryable_exceptions as e:
                    logger.error(f"{func.__name__} failed with non-retryable error: {e}")
                    if on_failure:
                        on_failure(e)
                    raise
                    
                except retryable_exceptions as e:
                    last_exception = e
                    
                    if attempt == max_attempts:
                        logger.error(f"{func.__name__} failed after {max_attempts} attempts: {e}")
                        if on_failure:
                            on_failure(e)
                        raise
                    
                    delay = exponential_backoff(
                        attempt, 
                        base_delay=base_delay, 
                        max_delay=max_delay, 
                        jitter=jitter
                    )
                    
                    logger.warning(f"{func.__name__} attempt {attempt} failed: {e}. Retrying in {delay:.2f}s...")
                    
                    if on_retry:
                        on_retry(attempt, e)
                    
                    time.sleep(delay)
                    
                except Exception as e:
                    # Unexpected exception - decide whether to retry
                    logger.error(f"{func.__name__} failed with unexpected error: {e}")
                    if on_failure:
                        on_failure(e)
                    raise
            
            # This should never be reached, but just in case
            if last_exception:
                raise last_exception
                
        return wrapper
    return decorator


def retry_on_failure(
    retries: int = 3,
    delay: float = 1.0,
    exceptions: Tuple[Type[Exception], ...] = RETRYABLE_EXCEPTIONS
):
    """
    Simple retry decorator for quick use cases.
    
    Args:
        retries: Number of retries
        delay: Fixed delay between retries
        exceptions: Exceptions to retry on
        
    Returns:
        Decorated function
    """
    return retry(
        max_attempts=retries + 1,
        base_delay=delay,
        backoff_multiplier=1.0,  # Fixed delay
        jitter=False,
        retryable_exceptions=exceptions
    )


class RetryContext:
    """
    Context manager for retry operations with custom logic.
    """
    
    def __init__(
        self,
        max_attempts: int = 3,
        base_delay: float = 1.0,
        max_delay: float = 60.0,
        retryable_exceptions: Tuple[Type[Exception], ...] = RETRYABLE_EXCEPTIONS
    ):
        self.max_attempts = max_attempts
        self.base_delay = base_delay
        self.max_delay = max_delay
        self.retryable_exceptions = retryable_exceptions
        self.attempt = 0
        self.last_exception = None
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type and issubclass(exc_type, self.retryable_exceptions):
            self.last_exception = exc_val
            self.attempt += 1
            
            if self.attempt < self.max_attempts:
                delay = exponential_backoff(self.attempt, self.base_delay, self.max_delay)
                logger.warning(f"Attempt {self.attempt} failed: {exc_val}. Retrying in {delay:.2f}s...")
                time.sleep(delay)
                return True  # Suppress the exception
        
        return False  # Let the exception propagate
    
    def should_retry(self) -> bool:
        """Check if we should retry the operation."""
        return self.attempt < self.max_attempts


def with_circuit_breaker(
    failure_threshold: int = 5,
    recovery_timeout: float = 60.0,
    expected_exception: Type[Exception] = Exception
):
    """
    Circuit breaker pattern decorator to prevent cascading failures.
    
    Args:
        failure_threshold: Number of failures before opening circuit
        recovery_timeout: Time to wait before attempting recovery
        expected_exception: Exception type that triggers circuit breaker
        
    Returns:
        Decorated function
    """
    class CircuitBreakerState:
        def __init__(self):
            self.failure_count = 0
            self.last_failure_time = 0
            self.state = 'CLOSED'  # CLOSED, OPEN, HALF_OPEN
    
    state = CircuitBreakerState()
    
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs):
            now = time.time()
            
            # Check if we should attempt recovery
            if (state.state == 'OPEN' and 
                now - state.last_failure_time > recovery_timeout):
                state.state = 'HALF_OPEN'
                logger.info(f"Circuit breaker for {func.__name__} entering HALF_OPEN state")
            
            # If circuit is open, fail fast
            if state.state == 'OPEN':
                raise NonRetryableError(f"Circuit breaker OPEN for {func.__name__}")
            
            try:
                result = func(*args, **kwargs)
                
                # Success - reset circuit breaker
                if state.state == 'HALF_OPEN':
                    state.state = 'CLOSED'
                    state.failure_count = 0
                    logger.info(f"Circuit breaker for {func.__name__} reset to CLOSED")
                
                return result
                
            except expected_exception as e:
                state.failure_count += 1
                state.last_failure_time = now
                
                if state.failure_count >= failure_threshold:
                    state.state = 'OPEN'
                    logger.error(f"Circuit breaker OPEN for {func.__name__} after {failure_threshold} failures")
                
                raise
        
        return wrapper
    return decorator


# Pre-configured retry decorators for common use cases
network_retry = retry(
    max_attempts=3,
    base_delay=2.0,
    max_delay=30.0,
    retryable_exceptions=(
        paramiko.SSHException,
        socket.timeout,
        socket.error,
        ConnectionResetError,
        ConnectionRefusedError,
    )
)

ssh_retry = retry(
    max_attempts=3,
    base_delay=1.0,
    max_delay=10.0,
    retryable_exceptions=(
        paramiko.SSHException,
        socket.timeout,
        BrokenPipeError,
    ),
    non_retryable_exceptions=(
        paramiko.AuthenticationException,
        paramiko.BadAuthenticationType,
    )
)

file_transfer_retry = retry(
    max_attempts=5,
    base_delay=1.0,
    max_delay=60.0,
    retryable_exceptions=(
        paramiko.SFTPError,
        paramiko.SSHException,
        socket.timeout,
        IOError,
        OSError,
    )
)