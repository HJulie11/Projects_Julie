from website.models import ClassificationData
import pytest
from website.routes import serialize_classification_data
from website import create_app

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

"""
CHECKING FOR VALID REQUESTS, ENSURING HANDLED CORRECTLY
"""

def test_create_classification(client):
    image_name = "random.jpg"
    item_class = "random"
    item_confidence = '0.9'
    bin_id = 1
    classification_data = ClassificationData(image_name=image_name, item_class=item_class, item_confidence=item_confidence, bin_id=bin_id)

    assert classification_data.image_name == "random.jpg"
    assert classification_data.item_class == "random"
    assert classification_data.item_confidence == '0.9'
    assert classification_data.bin_id == 1

def test_serialize_classification():
    image_name = "random.jpg"
    item_class = "random"
    item_confidence = '0.9'
    bin_id = 1
    classification_data = ClassificationData(image_name=image_name, item_class=item_class, item_confidence=item_confidence, bin_id=bin_id)
    
    assert serialize_classification_data(classification_data) == {"image_name": classification_data.image_name, "item_class": classification_data.item_class, "item_confidence": classification_data.item_confidence, "bin_id": classification_data.bin_id, "id": None}

def test_register_classification_data(client, mocker):
    image_name = "random.jpg"
    item_class = "random"
    item_confidence = '0.9'
    bin_id = 1
    classification_data = ClassificationData(image_name=image_name, item_class=item_class, item_confidence=item_confidence, bin_id=bin_id)

    mocker.patch("website.routes.create_classification_data", return_value=classification_data)
    res = client.post('/api/register_classification_data', json={"image_name":image_name, "item_class":item_class, "item_confidence":item_confidence, "bin_id":bin_id})

    assert res.status_code == 200
    assert res.json == {"image_name": classification_data.image_name, "item_class": classification_data.item_class, "item_confidence": classification_data.item_confidence, "bin_id": classification_data.bin_id, "id": None}

def test_delete_classifcation_data(client, mocker):
    image_name = "random.jpg"
    item_class = "random"
    item_confidence = '0.9'
    bin_id = 1
    mocker.patch("website.routes.delete_classification_data", return_value=None)
    res = client.post('/api/delete_classification_data', json={"id":1})
    assert res.status_code == 200
    assert res.json == {"message": "Classification data deleted"}


"""
CHECKING FOR INVALID REQUESTS, ENSURING HANDLED CORRECTLY
"""

def test_register_user_invalid_request(client):
    res = client.get('/api/register_classification_data')
    assert res.status_code == 405

def test_delete_user_invalid_request(client):
    res = client.get('/api/delete_classification_data')
    assert res.status_code == 405

