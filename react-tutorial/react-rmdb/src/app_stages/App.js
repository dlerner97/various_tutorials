import React from 'react';

// Components
import Header from '../components/Header';

// Styles
import { GlobalStyle } from '../GlobalStyle'

function App() { 
  return (
    <div className="App">
      <Header />
      Hello Dani!
      {/*Using the global style! */}
      <GlobalStyle /> 
    </div>
  );
}

export default App;
