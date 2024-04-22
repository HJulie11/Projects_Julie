import pytest
from website.routes import serialize_classification_data
from website import create_app
from website import db

@pytest.fixture()
def app():
    DB_NAME = "fake.db"
    app = create_app()
    app.config.update({
        "TESTING": True,
        "SQLALCHEMY_DATABASE_URI": f'sqlite:///{DB_NAME}',
        "SQLALCHEMY_TRACK_MODIFICATIONS": False
    })

    # Create the database file
    with app.app_context():
        db.create_all()
        print('Created Database!')

    yield app

    # Delete the database file after the test is complete
    # os.remove(f'../../instance/{DB_NAME}')


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
    item_confidence = 0.9
    bin_id = 1
    
    res = client.post('/api/register_classification_data', json={"image_name":image_name, "item_class":item_class, "item_confidence":item_confidence, "bin_id":bin_id})
    print(res.json)
    assert res.status_code == 200
    assert res.json == {"image_name": image_name, "item_class": item_class, "item_confidence": str(item_confidence), "bin_id": bin_id, "id": 1}

    