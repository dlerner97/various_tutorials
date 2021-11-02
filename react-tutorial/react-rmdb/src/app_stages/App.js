import React from 'react';

// Components
import Header from '../components/Header';
import Home from '../components/Home';

// Styles
import { GlobalStyle } from '../GlobalStyle'

function App() { 
  return (
    <div className="App">
      <Header />
      <Home />
      Hello Dani!
      {/*Using the global style! */}
      <GlobalStyle /> 
    </div>
  );
}

export default App;
