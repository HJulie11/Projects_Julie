from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support import expected_conditions as EC
from selenium.webdriver.support.ui import WebDriverWait
from selenium.common.exceptions import TimeoutException

def get_waitrose_products(search_word):
    driver = webdriver.Firefox()
    driver.get(f"https://www.waitrose.com/ecom/shop/search?&searchTerm={search_word}")
    try:
        WebDriverWait(driver,20).until(EC.element_to_be_clickable((By.XPATH, "/html/body/div[4]/div/div/div/section/div[2]/button[2]/span"))).click()
    except TimeoutException:
        pass
    waitrose_products = [element.text for element in driver.find_elements(By.XPATH, "//span[@class='name___STajL ellipses___dLChK']")]
    waitrose_prices = [element.text for element in driver.find_elements(By.XPATH, "//span[@class='itemPrice___j1MYI']")]
    filtered_waitrose_prices = [p.split('\n')[1] for p in waitrose_prices]
    filtered_waitrose_prices = [p.replace(' each est.', '') for p in filtered_waitrose_prices]
    driver.quit()
    return waitrose_products, filtered_waitrose_prices

def get_aldi_products(search_word):
    driver = webdriver.Firefox()
    driver.get(f"https://groceries.aldi.co.uk/en-GB/Search?keywords={search_word}")
    try:
        WebDriverWait(driver,20).until(EC.element_to_be_clickable((By.XPATH, "//*[@id='onetrust-accept-btn-handler']"))).click()
    except TimeoutException:
        pass
    aldi_products = [element.text for element in driver.find_elements(By.XPATH, "//*[contains(@class, 'product-tile-text')]")]
    aldi_prices = [element.text for element in driver.find_elements(By.XPATH, "//*[contains(@class, 'product-tile-price')]")]
    filtered_prices = [p.split('\n')[0] for p in aldi_prices]
    driver.quit()
    return aldi_products, filtered_prices

def get_tesco_products(search_word):
    driver = webdriver.Firefox()
    product_names = []
    prices = []
    driver.get(f"https://www.tesco.com/groceries/en-GB/search?query={search_word}")
    products = driver.find_elements(By.CSS_SELECTOR, '[data-auto="product-tile"]')
    for product in products:
        try:
            # Attempt to find the price within this product tile
            product_name = product.find_element(By.CSS_SELECTOR, '[data-auto="product-tile--title"]').text
            price = product.find_element(By.XPATH, ".//*[contains(@class, 'lnaeiZ beans-price__text')]").text
            product_names.append(product_name)
            prices.append(price)
        except:
            continue
    driver.quit()
    return product_names, prices
        
def get_all_products(search_word):
    waitrose_products, waitrose_prices = get_waitrose_products(search_word)
    aldi_products, aldi_prices = get_aldi_products(search_word)
    tesco_products, tesco_prices = get_tesco_products(search_word)
    return waitrose_products, waitrose_prices, aldi_products, aldi_prices, tesco_products, tesco_prices