# PhotoPi/tests/test_retry_utils.py
"""
Unit tests for retry utilities module.
"""

import pytest
import time
from unittest.mock import Mock, patch
import paramiko
import socket

from photopack.main_system.retry_utils import (
    exponential_backoff,
    retry,
    retry_on_failure,
    RetryContext,
    with_circuit_breaker,
    RetryableError,
    NonRetryableError,
    RETRYABLE_EXCEPTIONS,
    NON_RETRYABLE_EXCEPTIONS
)


class TestExponentialBackoff:
    """Test the exponential_backoff function."""
    
    def test_basic_backoff(self):
        """Test basic exponential backoff calculation."""
        delay1 = exponential_backoff(1, base_delay=1.0, jitter=False)
        delay2 = exponential_backoff(2, base_delay=1.0, jitter=False)
        delay3 = exponential_backoff(3, base_delay=1.0, jitter=False)
        
        assert delay1 == 1.0
        assert delay2 == 2.0
        assert delay3 == 4.0
    
    def test_max_delay_limit(self):
        """Test maximum delay limit."""
        delay = exponential_backoff(10, base_delay=1.0, max_delay=5.0, jitter=False)
        assert delay <= 5.0
    
    def test_jitter_variation(self):
        """Test that jitter adds variation."""
        delays = []
        for _ in range(10):
            delay = exponential_backoff(3, base_delay=1.0, jitter=True)
            delays.append(delay)
        
        # With jitter, delays should vary
        assert len(set(delays)) > 1
        # All delays should be positive
        assert all(d >= 0 for d in delays)
    
    def test_zero_attempt(self):
        """Test handling of zero attempt number."""
        delay = exponential_backoff(0, base_delay=1.0, jitter=False)
        assert delay >= 0


class TestRetryDecorator:
    """Test the retry decorator."""
    
    def test_successful_function_no_retry(self):
        """Test that successful functions don't trigger retries."""
        call_count = 0
        
        @retry(max_attempts=3)
        def successful_function():
            nonlocal call_count
            call_count += 1
            return "success"
        
        result = successful_function()
        assert result == "success"
        assert call_count == 1
    
    def test_function_succeeds_after_retries(self):
        """Test function that succeeds after some failures."""
        call_count = 0
        
        @retry(max_attempts=3, base_delay=0.01)
        def flaky_function():
            nonlocal call_count
            call_count += 1
            if call_count < 3:
                raise RetryableError("Temporary failure")
            return "success"
        
        result = flaky_function()
        assert result == "success"
        assert call_count == 3
    
    def test_function_fails_all_retries(self):
        """Test function that fails all retry attempts."""
        call_count = 0
        
        @retry(max_attempts=3, base_delay=0.01)
        def always_failing_function():
            nonlocal call_count
            call_count += 1
            raise RetryableError("Always fails")
        
        with pytest.raises(RetryableError):
            always_failing_function()
        assert call_count == 3
    
    def test_non_retryable_exception(self):
        """Test that non-retryable exceptions are not retried."""
        call_count = 0
        
        @retry(max_attempts=3)
        def non_retryable_function():
            nonlocal call_count
            call_count += 1
            raise NonRetryableError("Should not retry")
        
        with pytest.raises(NonRetryableError):
            non_retryable_function()
        assert call_count == 1
    
    def test_ssh_authentication_error_not_retried(self):
        """Test that SSH authentication errors are not retried."""
        call_count = 0
        
        @retry(max_attempts=3)
        def auth_error_function():
            nonlocal call_count
            call_count += 1
            raise paramiko.AuthenticationException("Bad credentials")
        
        with pytest.raises(paramiko.AuthenticationException):
            auth_error_function()
        assert call_count == 1
    
    def test_ssh_exception_is_retried(self):
        """Test that SSH exceptions are retried."""
        call_count = 0
        
        @retry(max_attempts=3, base_delay=0.01)
        def ssh_error_function():
            nonlocal call_count
            call_count += 1
            if call_count < 3:
                raise paramiko.SSHException("Connection lost")
            return "success"
        
        result = ssh_error_function()
        assert result == "success"
        assert call_count == 3
    
    def test_on_retry_callback(self):
        """Test the on_retry callback function."""
        retry_attempts = []
        
        def on_retry_callback(attempt, exception):
            retry_attempts.append((attempt, str(exception)))
        
        @retry(max_attempts=3, base_delay=0.01, on_retry=on_retry_callback)
        def flaky_function():
            if len(retry_attempts) < 2:
                raise RetryableError(f"Failure {len(retry_attempts) + 1}")
            return "success"
        
        result = flaky_function()
        assert result == "success"
        assert len(retry_attempts) == 2
        assert retry_attempts[0][0] == 1
        assert "Failure 1" in retry_attempts[0][1]
    
    def test_on_failure_callback(self):
        """Test the on_failure callback function."""
        failure_exceptions = []
        
        def on_failure_callback(exception):
            failure_exceptions.append(str(exception))
        
        @retry(max_attempts=2, base_delay=0.01, on_failure=on_failure_callback)
        def always_failing_function():
            raise RetryableError("Always fails")
        
        with pytest.raises(RetryableError):
            always_failing_function()
        assert len(failure_exceptions) == 1
        assert "Always fails" in failure_exceptions[0]


class TestRetryOnFailure:
    """Test the retry_on_failure decorator."""
    
    def test_simple_retry(self):
        """Test simple retry behavior."""
        call_count = 0
        
        @retry_on_failure(retries=2, delay=0.01)
        def flaky_function():
            nonlocal call_count
            call_count += 1
            if call_count < 3:
                raise socket.error("Network error")
            return "success"
        
        result = flaky_function()
        assert result == "success"
        assert call_count == 3
    
    def test_fixed_delay(self):
        """Test that retry_on_failure uses fixed delays."""
        delays = []
        original_sleep = time.sleep
        
        def mock_sleep(delay):
            delays.append(delay)
        
        with patch('time.sleep', side_effect=mock_sleep):
            @retry_on_failure(retries=2, delay=1.0)
            def failing_function():
                if len(delays) < 2:
                    raise socket.error("Network error")
                return "success"
            
            result = failing_function()
            assert result == "success"
            assert len(delays) == 2
            assert all(d == 1.0 for d in delays)  # Fixed delay


class TestRetryContext:
    """Test the RetryContext context manager."""
    
    def test_successful_operation(self):
        """Test successful operation without retries."""
        with RetryContext(max_attempts=3) as ctx:
            result = "success"
        
        assert ctx.attempt == 0
        assert result == "success"
    
    def test_retryable_exception(self):
        """Test handling of retryable exceptions."""
        attempt_count = 0
        
        for attempt in range(3):
            with RetryContext(max_attempts=3, base_delay=0.01) as ctx:
                attempt_count += 1
                if attempt_count < 3:
                    raise RetryableError("Temporary failure")
                result = "success"
                break
        
        assert result == "success"
        assert attempt_count == 3
    
    def test_should_retry_method(self):
        """Test the should_retry method."""
        ctx = RetryContext(max_attempts=3)
        
        assert ctx.should_retry() is True
        ctx.attempt = 2
        assert ctx.should_retry() is True
        ctx.attempt = 3
        assert ctx.should_retry() is False


class TestCircuitBreaker:
    """Test the circuit breaker decorator."""
    
    def test_circuit_breaker_opens_after_failures(self):
        """Test that circuit breaker opens after threshold failures."""
        call_count = 0
        
        @with_circuit_breaker(failure_threshold=3, recovery_timeout=0.1)
        def failing_function():
            nonlocal call_count
            call_count += 1
            raise Exception("Always fails")
        
        # First 3 calls should fail normally
        for _ in range(3):
            with pytest.raises(Exception):
                failing_function()
        
        # 4th call should fail with circuit breaker
        with pytest.raises(NonRetryableError):
            failing_function()
        
        assert call_count == 3  # Circuit breaker prevented 4th call
    
    def test_circuit_breaker_recovery(self):
        """Test circuit breaker recovery after timeout."""
        call_count = 0
        
        @with_circuit_breaker(failure_threshold=2, recovery_timeout=0.01)
        def sometimes_failing_function():
            nonlocal call_count
            call_count += 1
            if call_count <= 2:
                raise Exception("Initial failures")
            return "success"
        
        # Trigger circuit breaker
        for _ in range(2):
            with pytest.raises(Exception):
                sometimes_failing_function()
        
        # Circuit should be open
        with pytest.raises(NonRetryableError):
            sometimes_failing_function()
        
        # Wait for recovery timeout
        time.sleep(0.02)
        
        # Should succeed after recovery
        result = sometimes_failing_function()
        assert result == "success"
    
    def test_circuit_breaker_reset_on_success(self):
        """Test that circuit breaker resets on successful call."""
        call_count = 0
        failure_count = 0
        
        @with_circuit_breaker(failure_threshold=3, recovery_timeout=0.01)
        def intermittent_function():
            nonlocal call_count, failure_count
            call_count += 1
            if failure_count < 2:
                failure_count += 1
                raise Exception("Temporary failure")
            return "success"
        
        # First two calls fail
        for _ in range(2):
            with pytest.raises(Exception):
                intermittent_function()
        
        # Third call succeeds, should reset circuit breaker
        result = intermittent_function()
        assert result == "success"
        
        # Circuit breaker should be reset, so new failures should be allowed
        failure_count = 0  # Reset for new test
        with pytest.raises(Exception):
            intermittent_function()


class TestExceptionClassification:
    """Test exception classification for retries."""
    
    def test_retryable_exceptions(self):
        """Test that expected exceptions are classified as retryable."""
        retryable = [
            paramiko.SSHException("Connection lost"),
            socket.timeout("Timeout"),
            socket.error("Network error"),
            ConnectionResetError("Connection reset"),
            RetryableError("Custom retryable")
        ]
        
        for exc in retryable:
            assert isinstance(exc, RETRYABLE_EXCEPTIONS)
    
    def test_non_retryable_exceptions(self):
        """Test that expected exceptions are classified as non-retryable."""
        non_retryable = [
            paramiko.AuthenticationException("Bad auth"),
            paramiko.BadAuthenticationType("Bad auth type"),
            NonRetryableError("Custom non-retryable"),
            KeyboardInterrupt(),
            SystemExit()
        ]
        
        for exc in non_retryable:
            assert isinstance(exc, NON_RETRYABLE_EXCEPTIONS)


if __name__ == "__main__":
    pytest.main([__file__])