# streamlit run interactive_demo.py

from constants import *
import folium as fl
from streamlit_folium import st_folium
import streamlit as st
import serial

# function to open serial, will only actually run the first time
@st.cache_resource
def open_serial():
    print("opening serial")
    return serial.Serial(port=PORT, baudrate=BAUD)

# sets up webpage
m = fl.Map()
m.add_child(fl.LatLngPopup())
fl.CircleMarker(location=[DEFAULT_LAT, DEFAULT_LON], radius=2, weight=5).add_to(m)
map = st_folium(m, height=350, width=700)
altitude = st.slider("Altitude:", 0, 10000, 0)

# opens serial
ser = open_serial()

# gets coords and altitude if click registered
data = None
if map.get("last_clicked"):
    data = (map["last_clicked"]["lat"], map["last_clicked"]["lng"], altitude)

# if we have new data, send over serial
if data is not None:
    st.write(data)

    # serialize data, sent over serial
    serialized_data = bytearray([])
    for val in data:
        coord = int(val*SERIAL_SCALAR)
        for i in range(8):
            serialized_data.append(coord&0xFF)
            coord = coord >> 8

    ser.write(serialized_data)
