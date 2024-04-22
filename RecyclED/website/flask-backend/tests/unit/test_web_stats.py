from website.models import ClassificationData
import pytest
from website import create_app
import random
import datetime

@pytest.fixture()
def app():
    DB_NAME = "fake.db"
    app = create_app()
    app.config.update({
        "TESTING": True,
        # "SQLALCHEMY_DATABASE_URI": f'sqlite:///{DB_NAME}'
    })

    yield app

@pytest.fixture()
def client(app):
    with app.test_client() as client:
        client.app_context = app.app_context()
        client.app_context.push()
        yield client

@pytest.fixture()
def app_context(app):
    with app.app_context():
        yield


def test_pi_chart_data(client, mocker):
    
    recycled_data_example = ClassificationData(image_name="test.jpg", item_class="recycled", item_confidence=0.9, bin_id=1)
    general_waste_example = ClassificationData(image_name="test.jpg", item_class="general waste", item_confidence=0.9, bin_id=1)
    waste_arr = []
    for i in range(10):
        waste_arr.append(recycled_data_example)
        waste_arr.append(general_waste_example)    
    mocker.patch("website.routes.retrieve_items", return_value=waste_arr)

    response = client.get("api/pi_chart_data") 
    assert response.status_code == 200
    assert response.json == [{"name": "recycled", "value": 10}, {"name": "general waste", "value": 10}]


def test_pi_chart_data_empty(client, mocker):
    waste_arr = []
    mocker.patch("website.routes.retrieve_items", return_value=waste_arr)

    response = client.get("api/pi_chart_data")
    assert response.status_code == 200
    assert response.json == [{"name": "recycled", "value": 0}, {"name": "general waste", "value": 0}]


def test_pi_chart_data_random(client, mocker):
    recycled_data_example = ClassificationData(image_name="test.jpg", item_class="recycled", item_confidence=0.9, bin_id=1)
    general_waste_example = ClassificationData(image_name="test.jpg", item_class="general waste", item_confidence=0.9, bin_id=1)
    num_of_recycled = random.randint(0, 20)
    num_of_general = random.randint(0, 20)
    waste_arr = []
    for i in range(num_of_general):
        waste_arr.append(general_waste_example)
    for i in range(num_of_recycled):
        waste_arr.append(recycled_data_example)
    mocker.patch("website.routes.retrieve_items", return_value=waste_arr)

    response = client.get("api/pi_chart_data")
    assert response.status_code == 200
    assert response.json == [{"name": "recycled", "value": num_of_recycled}, {"name": "general waste", "value": num_of_general}]

def test_area_chart_data(client, mocker):
    test_data = []
    dates = []
    for i in range(7):
        date = datetime.date.today() - datetime.timedelta(days=i)
        date = date.strftime("%Y-%m-%d")
        dates.append(date)

    for date in dates:
        for i in range(10):
            test_data.append(ClassificationData(image_name="test.jpg", item_class="recycled", item_confidence=0.9, bin_id=1, time_created=f"Mon, {date} 12:00:00 GMT"))
            test_data.append(ClassificationData(image_name="test.jpg", item_class="general waste", item_confidence=0.9, bin_id=1, time_created=f"Mon, {date} 12:00:00 GMT"))

    mocker.patch("website.routes.retrieve_items", return_value=test_data)

    response = client.get("api/area_chart_data")
    assert response.status_code == 200
    #fix the line below
    expected = [{"date":  datetime.datetime.strptime(date, "%Y-%m-%d").strftime("%d/%m"), "general waste": 10, "recycled": 10} for date in reversed(dates)]
    assert response.json == expected


def test_area_chart_data_empty(client, mocker):
    test_data = []
    dates = []
    for i in range(7):
        date = datetime.date.today() - datetime.timedelta(days=i)
        date = date.strftime("%Y-%m-%d")
        dates.append(date)

    mocker.patch("website.routes.retrieve_items", return_value=test_data)

    response = client.get("api/area_chart_data")
    assert response.status_code == 200
    expected = [{"date":  datetime.datetime.strptime(date, "%Y-%m-%d").strftime("%d/%m"), "general waste": 0, "recycled": 0} for date in reversed(dates)]
    assert response.json == expected


def test_recycled_items(client, mocker):
    test_data = []
    recycled_data = []
    dates = []

    for i in range(7):
        date = datetime.date.today() - datetime.timedelta(days=i)
        date = date.strftime("%Y-%m-%d")
        dates.append(date)
    
    for date in dates:
        for i in range(10):
            test_data.append(ClassificationData(image_name="test.jpg", item_class="recycled", item_confidence=0.9, bin_id=1, time_created=f"Mon, {date} 12:00:00 GMT"))
            test_data.append(ClassificationData(image_name="test.jpg", item_class="general waste", item_confidence=0.9, bin_id=1, time_created=f"Mon, {date} 12:00:00 GMT"))

            recycled_data.append(ClassificationData(image_name="test.jpg", item_class="recycled", item_confidence=0.9, bin_id=1, time_created=f"Mon, {date} 12:00:00 GMT"))
    
    for i in range(5):
        test_data.append(ClassificationData(image_name="test.jpg", item_class="recycled", item_confidence=0.9, bin_id=1, time_created=f"Mon, 2022-04-05 12:00:00 GMT"))
        test_data.append(ClassificationData(image_name="test.jpg", item_class="recycled", item_confidence=0.9, bin_id=1, time_created=f"Mon, 2022-05-03 12:00:00 GMT"))
        test_data.append(ClassificationData(image_name="test.jpg", item_class="recycled", item_confidence=0.9, bin_id=1, time_created=f"Mon, 2022-06-02 12:00:00 GMT"))
        test_data.append(ClassificationData(image_name="test.jpg", item_class="recycled", item_confidence=0.9, bin_id=1, time_created=f"Mon, 2022-07-07 12:00:00 GMT"))

        recycled_data.append(ClassificationData(image_name="test.jpg", item_class="recycled", item_confidence=0.9, bin_id=1, time_created=f"Mon, 2022-04-05 12:00:00 GMT"))
        recycled_data.append(ClassificationData(image_name="test.jpg", item_class="recycled", item_confidence=0.9, bin_id=1, time_created=f"Mon, 2022-05-03 12:00:00 GMT"))
        recycled_data.append(ClassificationData(image_name="test.jpg", item_class="recycled", item_confidence=0.9, bin_id=1, time_created=f"Mon, 2022-06-02 12:00:00 GMT"))
        recycled_data.append(ClassificationData(image_name="test.jpg", item_class="recycled", item_confidence=0.9, bin_id=1, time_created=f"Mon, 2022-07-07 12:00:00 GMT"))
    
    mocker.patch("website.routes.retrieve_items", return_value=test_data)
    mocker.patch("website.routes.retrieve_item_by_class", return_value=recycled_data)

    response = client.get("api/recycled_items")
    assert response.status_code == 200
    expected = [{"proportion": round(len(recycled_data)/len(test_data), 2) , "recycled": len(recycled_data), "total":70}]
    assert response.json == expected


def test_recycled_items_empty(client, mocker):
    test_data = []
    recycled_data = []
    dates = []

    for i in range(7):
        date = datetime.date.today() - datetime.timedelta(days=i)
        date = date.strftime("%Y-%m-%d")
        dates.append(date)
    
    for date in dates:
        for i in range(10):
            test_data.append(ClassificationData(image_name="test.jpg", item_class="general waste", item_confidence=0.9, bin_id=1, time_created=f"Mon, {date} 12:00:00 GMT"))
    
    mocker.patch("website.routes.retrieve_items", return_value=test_data)
    mocker.patch("website.routes.retrieve_item_by_class", return_value=recycled_data)

    response = client.get("api/recycled_items")
    assert response.status_code == 200
    expected = [{"proportion": 0 , "recycled": 0, "total":0}]
    assert response.json == expected