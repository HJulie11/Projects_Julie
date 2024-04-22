from website.models import Bin, BinStatusEnum
import pytest
from website.routes import serialize_bin
from website import create_app
import json
from flask_sqlalchemy import SQLAlchemy

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

def test_create_bin():
    bin = Bin (user_id=1, classification_data=[], bin_status=BinStatusEnum.Green)
    assert bin.user_id == 1
    assert bin.classification_data == []
    assert bin.bin_status == BinStatusEnum.Green
    assert bin.bin_id == None

def test_serialize_bin():
    bin = Bin (user_id=1, classification_data=[], bin_status=BinStatusEnum.Green)
    assert serialize_bin(bin) == {"bin_id": bin.bin_id, "bin_status": json.dumps(bin.bin_status.value), "user_id": bin.user_id}

def test_register_bin(client, mocker):
    user_id = 1
    bin = Bin(user_id=user_id, classification_data=[], bin_status=BinStatusEnum.Green)
    mocker.patch("website.routes.create_bin", return_value=bin)

    res = client.post('/api/register_bin', json={"user_id":user_id})
    assert res.status_code == 200
    assert res.json == {"bin_id": bin.bin_id, "bin_status": json.dumps(bin.bin_status.value), "user_id": bin.user_id}

def test_read_bin(client, mocker):
    bin_id = 1
    bin = Bin(bin_id=bin_id, user_id=1, classification_data=[], bin_status=BinStatusEnum.Green)
    mocker.patch("website.routes.read_bin", return_value=bin)

    res = client.post('/api/read_bin', json={"bin_id":bin_id})
    assert res.status_code == 200
    assert res.json == {"bin_id": bin.bin_id, "bin_status": json.dumps(bin.bin_status.value), "user_id": bin.user_id}

def test_delete_bin(client, mocker):
    bin_id = 1
    mocker.patch("website.routes.delete_bin", return_value=None)

    res = client.post('/api/delete_bin', json={"bin_id":bin_id})
    assert res.status_code == 200
    assert res.json == {"message": "Bin deleted"}

def test_update_bin_status(client, mocker):
    bin_id = 1
    new_status = BinStatusEnum.Red
    mocker.patch("website.routes.update_bin_status", return_value=None)

    res = client.post('/api/update_bin_status', json={"bin_id":bin_id, "new_status":json.dumps(new_status.value)})
    assert res.status_code == 200
    assert res.json == {"message": "Bin status updated"}

"""
CHECKING FOR INVALID REQUESTS, ENSURING HANDLED CORRECTLY
"""

def test_read_bin_empty(client, mocker):
    bin_id = 1
    mocker.patch("website.routes.read_bin", return_value=None)

    res = client.post('/api/read_bin', json={"bin_id":bin_id})
    assert res.status_code == 401
    assert res.json == {"bin": None}

"""
CHECKING FOR INVALID REQUESTS, ENSURING HANDLED CORRECTLY
"""

def test_register_bin_invalid_request(client):
    res = client.get('/api/register_bin')
    assert res.status_code == 405

def test_delete_bin_invalid_request(client):
    res = client.get('/api/delete_bin')
    assert res.status_code == 405

def test_read_bin_invalid_request(client):
    res = client.get('/api/read_bin')
    assert res.status_code == 405

def test_update_bin_status_invalid_request(client):
    res = client.get('/api/update_bin_status')
    assert res.status_code == 405

