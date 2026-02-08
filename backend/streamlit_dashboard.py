import streamlit as st
import requests
import pandas as pd
import time

st.set_page_config(page_title="Heart Rate Monitor", page_icon="💓", layout="wide")

st.title("💓 Real-Time Heart Rate Monitor")

# Create placeholders
metrics_placeholder = st.empty()
chart_placeholder = st.empty()

# Auto-refresh loop
while True:
    try:
        # Fetch BPM history from FastAPI
        response = requests.get("http://localhost:8000/bpm_history", timeout=2)
        data = response.json()
        
        # Convert to DataFrame
        if data["data"]:
            df = pd.DataFrame(data["data"])
            current_bpm = data['current_bpm']
            total_readings = data['total_readings']
            elapsed = df['time'].max()
        else:
            df = pd.DataFrame(columns=['time', 'bpm'])
            current_bpm = 0
            total_readings = 0
            elapsed = 0
        
        # Update metrics (overwrite without clearing)
        with metrics_placeholder.container():
            col1, col2, col3 = st.columns(3)
            with col1:
                st.metric("Current BPM", f"{current_bpm:.1f}" if current_bpm > 0 else "---")
            with col2:
                st.metric("Total Readings", total_readings)
            with col3:
                st.metric("Elapsed Time", f"{int(elapsed)}s")
        
        # Update chart (overwrite without clearing)
        if not df.empty:
            chart_placeholder.line_chart(df.set_index('time')['bpm'], use_container_width=True)
        else:
            chart_placeholder.line_chart(pd.DataFrame({'bpm': []}), use_container_width=True)
    
    except requests.exceptions.RequestException:
        with metrics_placeholder.container():
            st.error("❌ Cannot connect to backend. Make sure FastAPI is running on port 8000")
    
    # Refresh every 2 seconds
    time.sleep(2)
    st.rerun()
