// Default names of files in an import statement is "index.js." So if we say, import "Header" from "./Header" where
// Header is the folder name, it'll assume the component is in the index.js which is why it's nice to use this naming
// convention. 
import React from "react";

import RMDBLogo from '../../images/react-movie-logo.svg';
import TMDBLogo from '../../images/tmdb_logo.svg';

import { Wrapper, Content, LogoImg, TMDBLogoImg } from './Header.styles' 

const Header = () => (
    <Wrapper>
        <Content>
            <LogoImg src={RMDBLogo} alt='rmdb-logo' />
            <TMDBLogo src={TMDBLogoImg} alt='tmdb-logo' />
        </Content>
    </Wrapper>
);

// Notice a lot of the time, we need to say import {...} from '...' The 'default' keyword tells the pkg manager which thing to import
// if a programmer simply types import ... from '...' 
export default Header;