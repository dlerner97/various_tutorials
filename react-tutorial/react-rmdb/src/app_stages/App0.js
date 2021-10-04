// First tutorial example. This just prints "Hello Dani!" to the screen.
import React from 'react';

// Alternate method for creating react elements in browser. Avoids using JSX (similar to html)
// const Star = () => React.createElement('div', null, 'This is a little star');

function App() { // Identical func declaration: "const App = () => {"
  return (
    <div className="App">
      Hello Dani!
    </div>
  );
}

export default App;