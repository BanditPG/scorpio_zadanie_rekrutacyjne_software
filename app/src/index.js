import React from 'react'
import ReactDOM from 'react-dom/client'

import './styles/index.css'
import './styles/motor.css'
import './styles/status.css'
import './styles/arm.css'

import App from './components/App'
import { RosProvider } from './contexts/RosConnectionContext'
import { RosMotorsProvider } from './contexts/RosMotorsContext'

const root = ReactDOM.createRoot(document.getElementById('root'))
root.render(
  <React.StrictMode>
    <RosProvider>
      <RosMotorsProvider>
        <App />
      </RosMotorsProvider>
    </RosProvider>
  </React.StrictMode>
)
