from website.models import User
from website import db

def create_user(email, password, org_name):
    user = User(email=email, password=password, org_name=org_name)
    try :
        db.session.add(user)
        db.session.commit()
        print("User created")
    except:
        print("Error in creating user, organisation or email already exists")
    return user

def read_user(email):
    try:
        return User.query.filter_by(email=email).first()
    except:
        print("Error in reading user")

def update_user(email, new_org_name, new_email, new_password):
    user = User.query.filter_by(email=email).first()
    if not user:
        print("Couldn't update user")
        return
    user.org_name = new_org_name
    user.email = new_email
    user.password = new_password
    db.session.commit()
    print("User updated")

def delete_user(email):
    user = User.query.filter_by(email=email).first()
    if not user:
        print("Couldn't delete user")
        return
    db.session.delete(user)
    db.session.commit()



