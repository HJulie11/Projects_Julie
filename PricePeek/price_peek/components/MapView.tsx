import React from 'react';

export default function MapView() {
    return (
        <iframe src="https://my.atlist.com/map/f79d30ae-f905-482c-a140-643c335a0403?share=true" 
                allow="geolocation 'self' https://my.atlist.com" 
                width="100%" 
                height="400px" 
                loading="lazy" 
                frameBorder="0" 
                scrolling="no" 
                allowFullScreen 
                id="atlist-embed">
        </iframe>
    );
}