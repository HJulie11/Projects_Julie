from . import db
from flask_login import UserMixin
from sqlalchemy.sql import func
from sqlalchemy import Enum
import enum


class BinStatusEnum(enum.Enum):
    Green = "Available"
    Amber = "Full"
    Red = "RequiresMaintenance"
    

class ClassificationData(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    image_name = db.Column(db.Text())
    item_class = db.Column(db.String(100))
    item_confidence = db.Column(db.String(100))
    time_created = db.Column(db.DateTime(timezone=True), server_default=func.now())
    bin_id = db.Column(db.Integer, db.ForeignKey('bin.bin_id'))
    
    
class Bin(db.Model):
    bin_id = db.Column(db.Integer, primary_key=True)
    bin_status = db.Column(Enum(BinStatusEnum), default=BinStatusEnum.Green)
    user_id = db.Column(db.Integer, db.ForeignKey('user.id'))
    classification_data = db.relationship('ClassificationData')


class User(db.Model, UserMixin):
    id = db.Column(db.Integer, primary_key=True)
    email = db.Column(db.String(150), unique=True)
    password = db.Column(db.String(150))
    org_name = db.Column(db.String(150), unique=True)
    bins = db.relationship('Bin')

