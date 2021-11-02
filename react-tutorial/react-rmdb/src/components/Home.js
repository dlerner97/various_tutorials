import React, { useState, useEffect } from 'react';

// API
import API from '../API'

// config
import { POSTER_SIZE, BACKDROP_SIZE, IMAGE_BASE_URL } from '../config';

// Components

// Hook

// Image
import NoImage from '../images/no_image.jpg'

const Home = () => {
    // bool value, setter
    const [state, setState] = useState()
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState(false);
    
    const fetchMovies = async (page, searchTerm = "") => {
        try {
            setError(false);
            setLoading(true);

            const movies = await API.fetchMovies(searchTerm, page);
            console.log(movies);

        } catch (error) {
            setError(true);
        }
    };

    // Inline function of initial render
    useEffect(() => {
        // Get me page one of movies
        fetchMovies(1)
    }, []) // Inside brackets is "dependency array." We will only use this func if dependencies are valid
           // If depend arr is empty, it'll only run once  

    return <div>Home Page</div>
}

export default Home;