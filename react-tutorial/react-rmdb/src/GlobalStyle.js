import { createGlobalStyle } from "styled-components";

// Exports our global style. I.e. when someone imports this script, they can get the global style guide we're selecting below.
// The `` denotes a template. Inside here, we're creating a global style guide. These styles are self-selected variable names
// using CSS notation. The nested sections
export const GlobalStyle = createGlobalStyle`
    /* The :root selector matches the docs root element. Basically, these are global vars within the webpage */
    :root {
        --maxWidth: 1280px;
        --white: #fff;
        --lightGray: #eee;
        --medGray: #353535;
        --darkGray: #1c1c1c;
        --fontHuge: 2.5rem;
        --fontBig: 1.5rem;
        --fontMed: 1.2rem;
        --fontSmall: 1rem;
    }

    /* "*" is the universal selector; it matches any element. So "*{...}" will apply the style rule to every element. */
    * {
        box-sizing: border-box;
        font-family: 'Abel', sans-serif;
    }

    /* Within a body, these are the css styles */
    body {
        margin: 0;
        padding: 0;

        /* Define headers 1, 3, and standard typing, p */
        h1 {
            font-size: 2rem;
            font-weight: 600;
            color: var(--white);
        }

        h3 {
            font-size: 1.1rem;
            font-weight: 600;
        }

        p {
            font-size: 1rem;
            color: var(--white);
        }
    }
`;