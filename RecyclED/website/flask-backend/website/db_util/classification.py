from website.models import ClassificationData
from website import db
import datetime

def create_classification_data(image_name, item_class, item_confidence, bin_id):
    classification_data = ClassificationData(image_name=image_name, item_class=item_class, item_confidence=item_confidence, bin_id=bin_id)
    try :
        db.session.add(classification_data)
        db.session.commit()
        print("classification_data created")
    except:
        print("Error in creating classification_data")
    return classification_data

def read_classification_data(bin_id):
    try:
        return ClassificationData.query.filter_by(bin_id=bin_id).all()
    except:
        print("Error in reading classification_data")

def delete_classification_data(id):
    classification_data = classification_data.query.filter_by(id=id).first()
    if not classification_data:
        print("Couldn't delete classification_data")
        return
    db.session.delete(classification_data)
    db.session.commit()
    print("classification_data deleted")

def retrieve_item_by_class(item_class):
    try:
        return ClassificationData.query.filter_by(item_class=item_class).all()
    except:
        print("Error in reading classification_data")
    
def retrieve_items():
    try:
        return ClassificationData.query.all()
    except:
        print("Error in reading classification_data")

def retrieve_item_by_date(date):
    try:
        items = ClassificationData.query.all()
        return items
    except:
        print("Error in reading classification_data")