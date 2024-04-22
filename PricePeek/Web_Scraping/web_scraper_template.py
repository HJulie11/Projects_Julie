from selenium import webdriver
from selenium.webdriver.chrome.service import Service
from selenium.webdriver.common.by import By
import sqlite3
import time

# Database setup
conn = sqlite3.connect('grocery_prices_dynamic.db')
c = conn.cursor()
c.execute('''CREATE TABLE IF NOT EXISTS prices
             (store TEXT, product_name TEXT, price TEXT)''')

def scrape_store(url, store_name, driver):
    # Navigate to the page
    driver.get(url)
    
    # Wait for JavaScript to load
    time.sleep(5)  # Adjust time according to your needs
    
    # Now you can parse the page as the content should be loaded
    products = driver.find_elements(By.XPATH, '//tagname[text()="Bread"]')
 # Update with correct class
    for product in products:
        # Extract data
        name = product.find_element(By.CLASS_NAME, 'product-name').text  # Update these selectors based on actual page structure
        price = product.find_element(By.CLASS_NAME, 'price').text  # Update these selectors based on actual page structure
        
        # Save data to database
        c.execute("INSERT INTO prices (store, product_name, price) VALUES (?, ?, ?)",
                  (store_name, name, price))
        conn.commit()

# Set up Selenium WebDriver
# service = Service('C:/path/to/chromedriver.exe')  # Update this path
driver = webdriver.Firefox()

# URLs and store names
stores = {
    'Tesco': 'https://www.tesco.com/groceries/en-GB/shop/bakery/bread-and-rolls/all',
}

for store, url in stores.items():
    print(f'Scraping {store}...')
    scrape_store(url, store, driver)

# Close the browser and database connection
driver.quit()
conn.close()
