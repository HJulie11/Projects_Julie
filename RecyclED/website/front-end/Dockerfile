#Node.js image
FROM node:alpine
#setting working directory
WORKDIR /front-end
#copying package.json file to the working directory
COPY package*.json /front-end
#installing the dependencies
RUN npm install
#copy the current directory contents to the container at /webapp
COPY . /front-end
#build the app
RUN npm run build
#expose the port 3000
EXPOSE 3000
#run the app
CMD npm start


