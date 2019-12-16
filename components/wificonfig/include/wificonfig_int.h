struct wificonfig_vals_wifi {
    char ap1_ssid[80]; // wifi_ap1_ssid
    char ap1_pswd[80]; // wifi_ap1_pswd
    char ap2_ssid[80]; // wifi_ap2_ssid
    char ap2_pswd[80]; // wifi_ap2_pswd
    char ap3_ssid[80]; // wifi_ap3_ssid
    char ap3_pswd[80]; // wifi_ap3_pswd
    char ap4_ssid[80]; // wifi_ap4_ssid
    char ap4_pswd[80]; // wifi_ap4_pswd
    char hostname[80]; // wifi_hostname
};

struct wificonfig_vals_mqtt {
    char host[20];    // mqtt_host
    uint16_t port;    // mqtt_port
    char client[80];  // mqtt_client
    char user[80];    // mqtt_user
    char pswd[80];    // mqtt_pswd
    char topic[80];   // mqtt_topic
    uint16_t update;  // mqtt_update
};

struct wificonfig_vals_rfid {
    char acl[20];       // rfid_acl
    uint8_t uniq;       // rfid_uniq
    char host[20];      // rfid_host
    uint16_t port;      // rfid_port
    uint8_t  cache;     // rfid_cache
    uint8_t  moment;    // rfid_moment
    uint16_t mtime;     // rfid_mtime
};
