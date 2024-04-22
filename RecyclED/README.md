**RecyclED?**

RecyclED is an autonomous recycling bin that sorts rubbish into right correct category (General or Recyclable). There is a cart moving along the rail with trapdoor to drop the item. The cart machanism is operated with object recognition system and motion detection system.

The project has an AI model trained with 1000 specific images of the most common waste within an office setting. Starting from this, the business has a high potentials in its scalability to other environments.

The modular design of the hardware model contributed to the scalability of the product, which implies that the users can use their own bins. 

<img src="../figs/product_last draft.png" width="350" />
<img src="../figs/product modular.png" width="350" />


**Logo**

<img src="../figs/RecylED_logo_wh.png" width="400"/>


**Website**

<video src="../figs/website video.mov" controls="controls" width = "max"></video>

**How to run web dev**

1. Run frontend dev first

    Go to '/front-end' folder
    ```
    cd /path/to/front-end
    ```

    run npm commands
    ```
    npm install
    ```
    ```
    npm run dev
    ```

2. Run backend server

    Go to '/flask-backend' folder
    ```
    cd /path/to/flask-backend
    ```

    Follow the commands below in order:
    ```
    python3 -m venv env
    ```
    ```
    source env/bin/activate
    ```
    (env) from here
    ```
    pip3 install -r requirements.txt
    ```
    ```
    python3 main.py
    ```
