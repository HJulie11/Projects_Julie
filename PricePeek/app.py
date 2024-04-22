
from Web_Scraping.web_scraper import get_all_products
from Database_Handling.database_write import DatabaseHandler
import json

def get_search_string():
    file_path = "data.json"
    with open(file_path, 'r') as file:
        search_str = json.load(file)
    return search_str
def main():
    #Testing data base creation
    input = get_search_string()
    waitrose_products, waitrose_prices, aldi_products, aldi_prices, tesco_products, tesco_prices = get_all_products(input)
    table_name = "prices_of_" + input
    database = DatabaseHandler(table_name)
    database.add_products_to_db(waitrose_products, waitrose_prices, "waitrose")
    database.add_products_to_db(aldi_products, aldi_prices, "aldi")
    database.add_products_to_db(tesco_products, tesco_prices, "tesco")
    print("Database created successfully")

if __name__ == "__main__":
    main()