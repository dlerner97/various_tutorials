// 2nd Tut script. Using states and styles.
// As of 10/4/2021, this script does not work... Will need to come back once I'm further along in the tutorial.
import React, { useState } from 'react';
import styled from 'styled-components'

const Square = styled.div`
  position: relative;
  width: 500px;
  height: 500px;
  border: 10px solid black;
  margin: 0 auto;
`;

function App() {

  // Creates a state called isApproaching in our App. NEVER change the state directly. Rather use the second parameter and
  // define a func as shown below
  const [isApproaching, setIsApproaching] = useState(false);

  // Toggles "isApproaching" when called. The argument is an inline function. When passing an inline function to setter from use state
  // it passes the previous state as the argument.
  const handleIsApproaching = () => setIsApproaching(prev => !prev);

  if (isApproaching) {
    return (
      <div className="App">
        <Square>
          Hello Dani!
        </Square>
      </div>
    )
  } else {
    return (
      <div className="App">
        <Square>
          Goodbye Dani!
        </Square>
      </div>
    )
  }
}

export default App;
