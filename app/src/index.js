import React from 'react';
import ReactDOM from 'react-dom/client';

import './styles/index.css';
import './styles/motor.css';
import './styles/status.css';

import App from './components/App';
// import reportWebVitals from './reportWebVitals';
import { RosProvider } from './contexts/RosConnectionContext';

const root = ReactDOM.createRoot(document.getElementById('root'));
root.render(
  <React.StrictMode>
    <RosProvider>
      <App />
    </RosProvider>
  </React.StrictMode>
);

// If you want to start measuring performance in your app, pass a function
// to log results (for example: reportWebVitals(console.log))
// or send to an analytics endpoint. Learn more: https://bit.ly/CRA-vitals
// reportWebVitals();
