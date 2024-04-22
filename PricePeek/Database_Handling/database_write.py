import sqlite3

class DatabaseHandler:
    def __init__(self, table_name):
        self.table_name = table_name
        self.conn = sqlite3.connect('grocery_prices_dynamic.db')
        self.c = self.conn.cursor()
        self.c.execute(f'''CREATE TABLE IF NOT EXISTS {self.table_name}
                 (store TEXT, product_name TEXT, price REAL)''')
        self.conn.commit()
        self.conn.close()

    def add_products_to_db(self, products, prices, store):
        self.conn = sqlite3.connect('grocery_prices_dynamic.db')
        self.c = self.conn.cursor()

        for product, price in zip(products, prices):
            if '£' in price:
                price = float(price.replace('£', ''))
            elif 'p' in price:
                price = float(price.replace('p', '')) / 100
            else:
                continue
            self.c.execute(f'''
            INSERT INTO {self.table_name} (store, product_name, price)
                        VALUES (?, ?, ?)
            ''', (store, product, price))

        self.conn.commit()
        self.conn.close()