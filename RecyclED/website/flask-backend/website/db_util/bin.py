from website.models import Bin
from website import db

def create_bin(user_id):
    bin = Bin(user_id=user_id)
    try :
        db.session.add(bin)
        db.session.commit()
        print("Bin created")
    except:
        print("Error in creating bin")
    return bin

def update_bin_status(bin_id, new_status):
    bin = Bin.query.filter_by(bin_id=bin_id).first()
    if not bin:
        print("Couldn't update bin status")
        return
    bin.bin_status = new_status
    db.session.commit()
    print("Bin status updated")

def delete_bin(bin_id):
    bin = Bin.query.filter_by(bin_id=bin_id).first()
    if not bin:
        print("Couldn't delete bin")
        return
    db.session.delete(bin)
    db.session.commit()
    print("Bin deleted")

def read_bin(bin_id):
    try:
        return Bin.query.filter_by(bin_id=bin_id).first()
    except:
        print("Error in reading bin")
    
