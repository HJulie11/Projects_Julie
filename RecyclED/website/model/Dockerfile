#selecting version of python
FROM python:3.9
#setting working directory
WORKDIR /model
COPY requirements.txt /model
#installing the dependencies
RUN pip install -r requirements.txt
#copy the current directory contents to the container at /flask-backend
COPY . /model
#expose the port 5000
EXPOSE 8080
#run the app
CMD python3 main.py



