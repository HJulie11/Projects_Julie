from website.models import User
from website.routes import serialize_user
import pytest
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

def test_create_user():
    """
    Given a User model
    When a new user is created
    Then check if the user is created
    """
    user = User(email="random@gmail.com", password="password", org_name="random")
    assert user.email == "random@gmail.com"
    assert user.password == "password"
    assert user.org_name == "random"

def test_serialize_user():
    user = User(email="siddharthsakriya@htomail.cim", password="password", org_name="siddharth")
    assert serialize_user(user=user) == {"id": user.id, "email": user.email, "org_name": user.org_name}

def test_register_user(client, mocker):
    email = "fake@gmail.com"
    password = "password"
    org_name = "siddharth"
    expected_user = User(email=email, password=password, org_name=org_name)

    mocker.patch("website.routes.create_user", return_value=expected_user)
    res = client.post('/api/register_user', json={"email":email, "password":password, "org_name":org_name})
    assert res.status_code == 200
    assert res.json == {"id": expected_user.id, "email": email, "org_name": org_name}

def test_read_user(client, mocker):
    email = "fake@gmail.com"
    password = "password"
    org_name = "siddharth"
    expected_user = User(email=email, password=password, org_name=org_name)

    mocker.patch('website.routes.read_user', return_value=expected_user)
    res = client.post('/api/read_user', json={"email":email})
    assert res.status_code == 200
    assert res.json == {"id": expected_user.id, "email": email, "org_name": org_name}

def test_delete_user(client, mocker):
    email = "fake@gmail.com"    
    mocker.patch('website.routes.delete_user', return_value=None)

    res = client.post('/api/delete_user', json={"email":email})
    assert res.status_code == 200
    assert res.json == {"message": "User deleted"}

def test_update_user(client, mocker):
    email = "fake@gmail.com"
    new_org_name = "siddharth"
    new_email = "sjsjsj@gmail.com"
    new_password = "password"
    mocker.patch('website.routes.update_user', return_value=None)

    res = client.post('/api/update_user', json={"email":email, "new_org_name":new_org_name, "new_email":new_email, "new_password":new_password})
    assert res.status_code == 200
    assert res.json == {"message": "User updated"}

def test_login_user(client, mocker):
    email = "fake@gmail.com"
    password = "password"
    org_name = "siddharth"
    expected_user = User(email=email, password=password, org_name=org_name)
    mocker.patch('website.routes.read_user', return_value=expected_user)

    res = client.post('/api/login_user', json={"email":email, "password":password})
    assert res.status_code == 200
    assert res.json == {"id": expected_user.id, "email": email, "org_name": org_name}
                

"""
CHECKING FOR INVALID REQUESTS, ENSURING HANDLED CORRECTLY
"""

def test_read_user_not_found(client, mocker):
    email = "fake@gmail.com"
    password = "password"
    org_name = "siddharth"
    expected_user = User(email=email, password=password, org_name=org_name)

    mocker.patch('website.routes.read_user', return_value=None)
    res = client.post('/api/read_user', json={"email":email})
    assert res.status_code == 401
    assert res.json == {"user": None}

def test_login_user_not_found(client, mocker):
    email = "fake@gmail.com"
    password = "password"
    org_name = "siddharth"

    mocker.patch('website.routes.read_user', return_value=None)
    res = client.post('/api/login_user', json={"email":email, "password":password})
    assert res.status_code == 401
    assert res.json == {"user": None}

def test_login_user_wrong_password(client, mocker):
    email = "dsbhjdshbds@gmail.com"
    password = "password"
    org_name = "dsbhsdhddjhs"
    user = User(email=email, password=password, org_name=org_name)
    
    mocker.patch('website.routes.read_user', return_value=user)
    res = client.post('/api/login_user', json={"email":email, "password":"wrongpass"})
    assert res.status_code == 401
    assert res.json == {"user": None}

"""
CHECKING FOR INVALID REQUESTS, ENSURING HANDLED CORRECTLY
"""

def test_register_user_invalid_request(client):
    res = client.get('/api/register_user')
    assert res.status_code == 405

def test_read_user_invalid_request(client):
    res = client.get('/api/read_user')
    assert res.status_code == 405

def test_update_user_invalid_request(client):
    res = client.get('/api/update_user')
    assert res.status_code == 405

def test_delete_user_invalid_request(client):
    res = client.get('/api/delete_user')
    assert res.status_code == 405

def test_login_user_invalid_request(client):
    res = client.get('/api/login_user')
    assert res.status_code == 405



