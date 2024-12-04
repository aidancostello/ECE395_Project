# streamlit run /Users/aidancostello/Github/ECE395_Project/python_scripts/interactive_demo.py

import folium as fl
from streamlit_folium import st_folium
import streamlit as st

DEFAULT_LAT = 40.115
DEFAULT_LON = -88.227778


def get_pos(lat, lng):
    return lat, lng

m = fl.Map()
m.add_child(fl.LatLngPopup())
fl.CircleMarker(location=[DEFAULT_LAT, DEFAULT_LON], radius=2, weight=5).add_to(m)

map = st_folium(m, height=350, width=700)

altitude = st.slider("Altitude:", 0, 10000, 0)

data = None
if map.get("last_clicked"):
    data = get_pos(map["last_clicked"]["lat"], map["last_clicked"]["lng"])

if data is not None:
    st.write((data[0], data[1], altitude)) # Writes to the app
    print((data[0], data[1], altitude)) # Writes to terminal
