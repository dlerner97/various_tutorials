// Custom hook

import API from '../API'
import { useState, useEffect, useRef } from 'react';

const initialState = {
    page: 0,
    results: [],
    total_pages: 0,
    total_results: 0
}

export const useHomeFetch = () => {
    // bool value, setter
    const [state, setState] = useState(initialState)
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState(false);
    
    const fetchMovies = async (page, searchTerm = "") => {
        try {
            setError(false);
            setLoading(true);

            const movies = await API.fetchMovies(searchTerm, page);

            // setState is a setter that uses an inline function (with prev as input) to reset the state 
            setState(prev => ({
                // ... syntax "spreads out" the movies var. Creates an new object with all of the properties of the given object
                ...movies,
                // returns all movies until now
                results:
                    page > 1 ? [...prev.results, ...movies.results] : [...movies.results]
            }))

        } catch (error) {
            setError(true);
        }
        setLoading(false);
    };

    // Inline function of initial render
    useEffect(() => {
        // Get me page one of movies
        fetchMovies(1)
    }, []) // Inside brackets is "dependency array." We will only use this func if dependencies are valid
        // If depend arr is empty, it'll only run once  
        return { state, loading, error }
}