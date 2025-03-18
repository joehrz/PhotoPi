import mysql.connector

def insert_metrics_to_mysql(row_data, db_config):
    """
    Inserts or updates a metrics row in a MySQL database.
    
    Parameters:
      row_data (dict): The metrics for a single file, including a unique 'Cultivar'.
      db_config (dict): A dictionary containing the MySQL connection parameters, e.g.:
        {
            "user": "your_user",
            "password": "your_password",
            "host": "localhost",
            "database": "your_database"
        }
    
    This function assumes your MySQL table is named 'metrics' and that the 'Cultivar'
    column is defined as UNIQUE.
    """
    try:
        conn = mysql.connector.connect(**db_config)
        cursor = conn.cursor()

        # Extract columns and values from row_data
        columns = list(row_data.keys())
        values = [row_data[col] for col in columns]

        placeholders = ", ".join(["%s"] * len(columns))
        col_names = ", ".join(columns)
        # Build update clause excluding the 'Cultivar' key (which is unique)
        update_clause = ", ".join([f"{col}=VALUES({col})" for col in columns if col != "Cultivar"])

        sql = f"""
            INSERT INTO metrics ({col_names})
            VALUES ({placeholders})
            ON DUPLICATE KEY UPDATE {update_clause}
        """
        cursor.execute(sql, values)
        conn.commit()
        logger.info("Inserted/Updated row for cultivar: " + row_data.get("Cultivar", "(unknown)"))
    except Exception as e:
        logger.error("Error inserting/updating row in MySQL: " + str(e))
        conn.rollback()
    finally:
        if cursor:
            cursor.close()
        if conn:
            conn.close()