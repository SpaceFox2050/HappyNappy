import streamlit as st
import requests
import pandas as pd
import time
import plotly.graph_objects as go

st.set_page_config(page_title="Happy Nappy", page_icon="😴", layout="wide")

# Custom CSS for better styling
st.markdown("""
    <style>
    .main {
        background-color: white;
    }
    .stMetric {
        background-color: #f8f9fa;
        padding: 15px;
        border-radius: 10px;
        box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
        border: 1px solid #dee2e6;
    }
    .stMetric label {
        color: #212529 !important;
    }
    .stMetric [data-testid="stMetricValue"] {
        color: #212529 !important;
    }
    </style>
""", unsafe_allow_html=True)

# Header with custom styling
st.markdown("""
    <div style='text-align: center; padding: 20px; background: white; 
                border-radius: 20px; margin-bottom: 30px; box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
                border: 2px solid #667eea;'>
        <h1 style='color: #212529; font-size: 3.5em; margin: 0; font-weight: bold;'>
            😴 Happy Nappy 💤
        </h1>
        <p style='color: #495057; font-size: 1.2em; margin-top: 10px;'>
            AI-Powered Power Nap Assistant
        </p>
    </div>
""", unsafe_allow_html=True)

# Create placeholders
metrics_placeholder = st.empty()
chart_placeholder = st.empty()
download_placeholder = st.empty()

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
                st.metric("💓 Current BPM", f"{current_bpm:.1f}" if current_bpm > 0 else "---", 
                         delta=None, delta_color="normal")
            with col2:
                st.metric("📊 Total Readings", total_readings)
            with col3:
                st.metric("⏱️ Elapsed Time", f"{int(elapsed // 60)}m {int(elapsed % 60)}s")
        
        # Update chart with Plotly (overwrite without clearing)
        if not df.empty:
            with chart_placeholder.container():
                st.markdown("### 📈 Heart Rate Trend")
                
                # Create Plotly figure
                fig = go.Figure()
                fig.add_trace(go.Scatter(
                    x=df['time'],
                    y=df['bpm'],
                    mode='lines+markers',
                    name='BPM',
                    line=dict(color='#667eea', width=3),
                    marker=dict(size=6, color='#764ba2'),
                    fill='tozeroy',
                    fillcolor='rgba(102, 126, 234, 0.1)'
                ))
                
                fig.update_layout(
                    xaxis_title="Time (seconds)",
                    yaxis_title="BPM",
                    hovermode='x unified',
                    plot_bgcolor='white',
                    paper_bgcolor='white',
                    font=dict(size=13, color='#212529'),
                    height=400,
                    margin=dict(l=50, r=50, t=30, b=50),
                    xaxis=dict(
                        gridcolor='#e9ecef',
                        linecolor='#dee2e6'
                    ),
                    yaxis=dict(
                        gridcolor='#e9ecef',
                        linecolor='#dee2e6'
                    )
                )
                
                st.plotly_chart(fig, use_container_width=True)
        else:
            chart_placeholder.info("⏳ Waiting for heart rate data...")
        
        # Download section
        with download_placeholder.container():
            if not df.empty:
                col1, col2, col3 = st.columns([1, 2, 1])
                with col2:
                    st.markdown("---")
                    csv_data = df.to_csv(index=False).encode('utf-8')
                    st.download_button(
                        label="📥 Download Heart Rate Data (CSV)",
                        data=csv_data,
                        file_name=f"happy_nappy_data_{int(time.time())}.csv",
                        mime="text/csv",
                        use_container_width=True
                    )
    
    except requests.exceptions.RequestException:
        with metrics_placeholder.container():
            st.markdown("""
                <div style='background-color: #ff6b6b; padding: 20px; border-radius: 10px; 
                            color: white; text-align: center;'>
                    <h3>❌ Cannot connect to backend</h3>
                    <p>Make sure FastAPI is running on port 8000</p>
                </div>
            """, unsafe_allow_html=True)
    
    # Refresh every 2 seconds
    time.sleep(2)
    st.rerun()
