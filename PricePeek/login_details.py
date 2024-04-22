import sqlite3

def create_connection(db_file):
    """Create a database connection to a SQLite database."""
    conn = None
    try:
        conn = sqlite3.connect(db_file)
        print(sqlite3.version)
    except sqlite3.Error as e:
        print(e)
    return conn

def create_table(conn, create_table_sql):
    try:
        c = conn.cursor()
        c.execute(create_table_sql)
    except sqlite3.Error as e:
        print(e)

def create_user(conn, user):
    sql = ''' INSERT INTO users(username,password)
              VALUES(?,?) '''
    cur = conn.cursor()
    cur.execute(sql, user)
    return cur.lastrowid

def main():
    database = "users.db"
    
    # SQL statement to create a table
    create_table_sql = """ CREATE TABLE IF NOT EXISTS users (
                            id INTEGER PRIMARY KEY AUTOINCREMENT,
                            username TEXT NOT NULL,
                            password TEXT NOT NULL
                        ); """
    
    # create a database connection
    conn = create_connection(database)
    if conn is not None:
        # create users table
        create_table(conn, create_table_sql)
    else:
        print("Error! cannot create the database connection.")
        return
    
    # example users
    # example users
    users = [('charvi_agarwal', 'charvi123'),
         ('amishi_gangwar', 'amishi456'),
         ('dorna_hamed_barghi', 'dorna@123'),
         ('roberta_posiunaite', 'roberta@456'),
         ('julie_hong', 'julie@123')]

    
    # insert example users into the database
    with conn:
        for user in users:
            create_user(conn, user)

if __name__ == '__main__':
    main()
