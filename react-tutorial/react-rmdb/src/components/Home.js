import React from 'react';

// API
import API from '../API';

// config
import { POSTER_SIZE, BACKDROP_SIZE, IMAGE_BASE_URL } from '../config';

// Hook
import { useHomeFetch } from '../hooks/useHomeFetch';

// Components
import HeroImage from './HeroImage';
import Grid from './Grid';
import Thumb from './Thumb';
import Spinner from './Spinner';

// Image
import NoImage from '../images/no_image.jpg';

const Home = () => {
    const { state, loading, error } = useHomeFetch();
    console.log(state);
    return (
        // Like CPP, js can only return one element. Here, we return a "fragment" similar to a div but doesn't seperate out this component.
        // Fragment notation is a simple <>. Furthermore, a "{}" in HTML converts any javascript output to HTML. Similar to f-string in python
        <>
        {/* If state.results[0] exists, create a hero image from this first result. Otherwise, send null  */}
            {state.results[0] ? (
                <HeroImage
                    // All vars in this section come from the movie database API
                    // Template literal. Similar to XML. Below, is the full image path
                    image={`${IMAGE_BASE_URL}${BACKDROP_SIZE}${state.results[0].backdrop_path}`}
                    title={state.results[0].original_title}
                    text={state.results[0].overview}
                />
            ) : null
            }
            <Grid header='Popular Movies'>
                {/* map is basically a range-based for loop */}
                {state.results.map(movie => (
                    <Thumb
                        key={movie.id}
                        clickable
                        image={
                            movie.poster_path ?
                                IMAGE_BASE_URL + POSTER_SIZE + movie.poster_path
                                : NoImage 
                        }
                        movieId={movie.id}
                    />
                ))}
            </Grid>
            <Spinner />
        </>        
    )
}

export default Home;