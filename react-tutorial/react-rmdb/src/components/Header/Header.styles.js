import styled from "styled-components";

// Style guid for anything in the wrapper div
export const Wrapper = styled.div`
    background: var(--darkGray);
    padding: 0 20px;
`;

// Style guide for anything in the component div
export const Content = styled.div`
    display: flex;
    align-items: center;
    justify-content: space-between;
    max-width: var(--maxWidth);
    padding: 20px 0;
    margin: 0 auto;
`;

// Style component for logo img
export const LogoImg = styled.img`
    width: 200px;
    @media screen and (max-width: 500px) {
        width: 150px;
    }
`;

// Style component for TMDB logo image
export const TMDBLogoImg = styled.img`
    width: 100px;
    @media screen and (max-width: 500px) {
        width: 80px;
    }
`;