import React from 'react';
import ReactDOM from 'react-dom';
import App from './app_stages/App';

// This file calls the App function from the App.js file. Basically the "main" as I understand it

ReactDOM.render(
  <React.StrictMode>
    <App />
  </React.StrictMode>,
  document.getElementById('root')
);
