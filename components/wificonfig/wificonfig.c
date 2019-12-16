/* wificonfig

Allows configuration of a device over wifi

*/

#include <string.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <esp_netif.h>
#include <esp_eth.h>

#include <esp_http_server.h>

#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <lwip/err.h>
#include <lwip/sys.h>

#include <wificonfig_int.h>

/* A simple example that demonstrates how to create GET and POST
 * handlers for the web server.
 */

#define WIFI_SSID     "ESP32"
#define WIFI_PASS     ""
#define MAX_STA_CONN  4
#define DEFAULT_SCAN_LIST_SIZE 10

#define D_TITLE       CONFIG_WIFI_PAGE_TITLE
#define D_DEVICE      CONFIG_WIFI_MODULE_NAME

#define D_NAMESPACE   "wificonfig"

struct wificonfig_vals_wifi wificonfig_vals_wifi;
struct wificonfig_vals_mqtt wificonfig_vals_mqtt;
struct wificonfig_vals_rfid wificonfig_vals_rfid;

extern const char *TAG;

const char *THIS_HTTP_STYLE =
    "<style>"
    "div,fieldset,input,select{padding:5px;font-size:1em;}"
    "fieldset{background:#4f4f4f;}p{margin:0.5em 0;}"
    "input{width:100%;box-sizing:border-box;-webkit-box-sizing:border-box;-moz-box-sizing:border-box;background:#dddddd;color:#000000;}"
    "input[type=checkbox],input[type=radio]{width:1em;margin-right:6px;vertical-align:-1px;}"
    "input[type=range]{width:99%;}"
    "select{width:100%;background:#dddddd;color:#000000;}"
    "textarea{resize:none;width:98%;height:318px;padding:5px;overflow:auto;background:#1f1f1f;color:#65c115;}"
    "body{text-align:center;font-family:verdana,sans-serif;background:#252525;}"
    "td{padding:0px;}"
    "button{border:0;border-radius:0.3rem;background:#1fa3ec;color:#faffff;line-height:2.4rem;font-size:1.2rem;width:100%;-webkit-transition-duration:0.4s;transition-duration:0.4s;cursor:pointer;}"
    "button:hover{background:#0e70a4;}"
    ".bred{background:#d43535;}"
    ".bred:hover{background:#931f1f;}"
    ".bgrn{background:#47c266;}"
    ".bgrn:hover{background:#5aaf6f;}"
    "a{color:#1fa3ec;text-decoration:none;}"
    ".p{float:left;text-align:left;}"
    ".q{float:right;text-align:right;}"
    ".r{border-radius:0.3em;padding:2px;margin:6px 2px;}"
    "</style>"
    ;

const char *THIS_HTTP_HOME_SCRIPT =
    "<script>"
    "var x=null,lt,to,tp,pc='';"
    "function eb(s){return document.getElementById(s);}"
    "function qs(s){return document.querySelector(s);}"
    "function sp(i){eb(i).type=(eb(i).type==='text'?'password':'text');}"
    "function wl(f){window.addEventListener('load',f);}"
    "function la(p){"
        "var a='';"
        "if(la.arguments.length==1){"
            "a=p;"
            "clearTimeout(lt);"
            "}"
        "if(x!=null){x.abort();}"
        "x=new XMLHttpRequest();"
        "x.onreadystatechange=function(){"
            "if(x.readyState==4 && x.status==200){"
                "var s=x.responseText.replace(/{t}/g,\"<table style='width:100%'>\").replace(/{s}/g,\"<tr><th>\").replace(/{m}/g,\"</th><td>\").replace(/{e}/g,\"</td></tr>\").replace(/{c}/g,\"%'><div style='text-align:center;font-weight:\");"
                "eb('l1').innerHTML=s;"
                "}"
            "};"
        "x.open('GET','.?m=1'+a,true);"
        "x.send();"
        "lt=setTimeout(la,2345);}"
    "function lc(v,i,p){"
        "if(v=='h'||v=='d'){"
            "var sl=eb('sl4').value;"
            "eb('s').style.background='linear-gradient(to right,rgb('+sl+'%,'+sl+'%,'+sl+'%),hsl('+eb('sl2').value+',100%,50%))';"
         "}"
         "la('&'+v+i+'='+p);"
    "}"
    "wl(la);"
    "function jd(){"
       "var t=0,i=document.querySelectorAll('input,button,textarea,select');"
       "while(i.length>=t){"
           "if(i[t]){"
                   "i[t]['name']=(i[t].hasAttribute('id')&&(!i[t].hasAttribute('name')))?i[t]['id']:i[t]['name'];"
           "}"
           "t++;"
       "}"
    "}"
    "wl(jd);"
    "</script>"
    ;

const char *THIS_HTTP_WIFI_SCRIPT =
    "<script>"
    "function eb(s){return document.getElementById(s);}"
    "function c(l){"
        "eb('s1').value=l.innerText||l.textContent;"
        "eb('p1').focus();"
        "}"
    "</script>"
    ;

const char *THIS_HTTP_HEAD_START =
    "<!DOCTYPE html>"
    "<html class=\"\">"
    "<head>"
    "<meta charset='utf-8'>"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1,user-scalable=no\"/>"
    "<title>" D_TITLE "</title>"
    ;

const char *THIS_HTTP_HEAD_END =
    "</head>"
    ;

const char *THIS_HTTP_BODY_START =
    "<body>"
    "<div style=\"text-align:left;display:inline-block;color:#eaeaea;min-width:340px;\">"
    ;


const char *THIS_HTTP_BODY_HOME = 
    "<div style=\"text-align:center;color:#eaeaea;\">"
    "<h3>Main Configuration</h3>"
    "<h2>" D_DEVICE "</h2>"
    "</div>"
    "<p></p><form action=\"wifi\" method=\"get\">"
    "<button name>Configure Wifi</button></form><p></p>"
    "<p></p><form action=\"mqtt\" method=\"get\">"
    "<button name>Configure MQTT</button></form><p></p>"
    "<p></p><form action=\"rfid\" method=\"get\">"
    "<button name>Configure RFID</button></form><p></p>"
    "<p></p><form action=\"save\" method=\"get\">"
    "<button name>Save Configuration</button></form><p></p>"
    "<p></p><form action=\"restart\" method=\"get\" onsubmit=\"return confirm(\'Confirm Restart\');\">"
    "<button name=\"restart\" class=\"button bred\">Restart</button></form><p></p>"
    ;

const char *THIS_HTTP_BODY_WIFI_0 = 
    "<div style=\"text-align:center;color:#eaeaea;\">"
    "<h3>Wifi Configuration</h3>"
    "<h2>" D_DEVICE "</h2>"
    "</div>"
    "<div>"
    ;

const char *THIS_HTTP_BODY_WIFI_SCAN_0 = 
    "<div>"
    "No wifi networks found"
    "</div>"
    "<br>"
    ;

const char *THIS_HTTP_BODY_WIFI_SCAN_1 = 
    "<div>"
    "<a href=\"#p\" onclick=\"c(this)\">"
    ;

const char *THIS_HTTP_BODY_WIFI_SCAN_2 = 
    "<span class=\"q\">"
    ;

const char *THIS_HTTP_BODY_WIFI_SCAN_3 = 
    "</span>"
    "</div>"
    ;

const char *THIS_HTTP_BODY_WIFI_1 = 
    "<fieldset>"
    "<legend><b>Wifi Parameters</b></legend>"
    "<form method=\"get\" action=\"wifi\">"
    "<p><b>AP1 SSId</b><br><input id=\"s1\" value=\""
    ;

const char *THIS_HTTP_BODY_WIFI_2 = 
    "\" name=\"s1\"></p>"
    "<p><b>AP1 Password</b><br><input id=\"p1\" value=\""
    ;

const char *THIS_HTTP_BODY_WIFI_3 = 
    "\" name=\"p1\"></p>"
    "<p><b>AP2 SSId</b><input id=\"s2\" value=\""
    ;

const char *THIS_HTTP_BODY_WIFI_4 = 
    "\" name=\"s2\"></p>"
    "<p><b>AP2 Password</b><br><input id=\"p2\" value=\""
    ;

const char *THIS_HTTP_BODY_WIFI_5 = 
    "\" name=\"p2\"></p>"
    "<p><b>AP3 SSId</b><input id=\"s3\" value=\""
    ;

const char *THIS_HTTP_BODY_WIFI_6 = 
    "\" name=\"s3\"></p>"
    "<p><b>AP3 Password</b><br><input id=\"p3\" value=\""
    ;

const char *THIS_HTTP_BODY_WIFI_7 = 
    "\" name=\"p3\"></p>"
    "<p><b>AP4 SSId</b><input id=\"s4\" value=\""
    ;

const char *THIS_HTTP_BODY_WIFI_8 = 
    "\" name=\"s4\"></p>"
    "<p><b>AP4 Password</b><br><input id=\"p4\" value=\""
    ;

const char *THIS_HTTP_BODY_WIFI_9 = 
    "\" name=\"p4\"></p>"
    "<p><b>Hostname</b><br><input id=\"hn\" value=\""
    ;

const char *THIS_HTTP_BODY_WIFI_10 = 
    "\" name=\"hn\"></p>"
    "<br><button name=\"save\" type=\"submit\" class=\"button bgrn\">Update</button>"
    "</form>"
    "</fieldset>"
    "<div></div>"
    "<p></p>"
    "<form action=\"/\" method=\"get\"><button name>Home</button></form>"
    ;

const char *THIS_HTTP_BODY_MQTT_0 = 
    "<div style=\"text-align:center;color:#eaeaea;\">"
    "<h3>MQTT Configuration</h3>"
    "<h2>" D_DEVICE "</h2>"
    ;

const char *THIS_HTTP_BODY_MQTT_1 = 
    "<fieldset>"
    "<legend><b>MQTT Parameters</b></legend>"
    "<form method=\"get\" action=\"mqtt\">"
    "<p><b>Host (leave blank to disable MQTT)</b><br><input id=\"ho\" value=\""
    ;

const char *THIS_HTTP_BODY_MQTT_2 = 
    "\" name=\"ho\"></p>"
    "<p><b>Port</b><br><input id=\"po\" value=\""
    ;

const char *THIS_HTTP_BODY_MQTT_3 = 
    "\" name=\"po\"></p>"
    "<p><b>Client</b><br><input id=\"cl\" value=\""
    ;

const char *THIS_HTTP_BODY_MQTT_4 = 
    "\" name=\"cl\"></p>"
    "<p><b>User</b><br><input id=\"us\" value=\""
    ;

const char *THIS_HTTP_BODY_MQTT_5 = 
    "\" name=\"us\"></p>"
    "<p><b>Password</b><br><input id=\"pa\" value=\""
    ;

const char *THIS_HTTP_BODY_MQTT_6 = 
    "\" name=\"pa\"></p>"
    "<p><b>Topic</b><br><input id=\"to\" value=\""
    ;

const char *THIS_HTTP_BODY_MQTT_7 = 
    "\" name=\"to\"></p>"
    "<p><b>Update Interval (minutes, 0 to disable)</b><br><input id=\"up\" value=\""
    ;

const char *THIS_HTTP_BODY_MQTT_8 = 
    "\" name=\"up\"></p>"
    "<br><button name=\"save\" type=\"submit\" class=\"button bgrn\">Update</button>"
    "</form>"
    "</fieldset>"
    "<div></div>"
    "<p></p>"
    "<form action=\"/\" method=\"get\"><button name>Home</button></form>"
    ;

const char *THIS_HTTP_BODY_RFID_0 = 
    "<div style=\"text-align:center;color:#eaeaea;\">"
    "<h3>RFID Configuration</h3>"
    "<h2>" D_DEVICE "</h2>"
    "</div>"
    ;

const char *THIS_HTTP_BODY_RFID_1 = 
    "<fieldset>"
    "<legend><b>RFID Parameters</b></legend>"
    "<form method=\"get\" action=\"rfid\">"
    "<p><b>ACL Name</b><br><input id=\"ac\" value=\""
    ;

const char *THIS_HTTP_BODY_RFID_2 = 
    "\" name=\"ac\"></p>"
    "<p><b>Instance</b><br><input id=\"un\" value=\""
    ;

const char *THIS_HTTP_BODY_RFID_3 = 
    "\" name=\"un\"></p>"
    "<p><b>ACL Server Host</b><br><input id=\"ho\" value=\""
    ;

const char *THIS_HTTP_BODY_RFID_4 = 
    "\" name=\"ho\"></p>"
    "<p><b>ACL Server Port</b><br><input id=\"po\" value=\""
    ;

const char *THIS_HTTP_BODY_RFID_5 = 
    "\" name=\"po\"></p>"
    "<p><b>Caching? (0/1)</b><br><input id=\"ca\" value=\""
    ;

const char *THIS_HTTP_BODY_RFID_6 = 
    "\" name=\"ca\"></p>"
    "<p><b>Momentary? (0/1)</b><br><input id=\"mo\" value=\""
    ;

const char *THIS_HTTP_BODY_RFID_7 = 
    "\" name=\"mo\"></p>"
    "<p><b>Momentary Time</b><br><input id=\"mt\" value=\""
    ;

const char *THIS_HTTP_BODY_RFID_8 = 
    "\" name=\"mt\"></p>"
    "<br><button name=\"save\" type=\"submit\" class=\"button bgrn\">Update</button>"
    "</form>"
    "</fieldset>"
    "<div></div>"
    "<p></p>"
    "<form action=\"/\" method=\"get\"><button name>Home</button></form>"
    ;

const char *THIS_HTTP_BODY_SAVE = 
    "<div style=\"text-align:center;color:#eaeaea;\">"
    "<h3>Save Configuration</h3>"
    "<h2>" D_DEVICE "</h2>"
    "</div>"
    "Configuration Saved. Restart to use saved settings."
    "<p></p>"
    "<form action=\"/\" method=\"get\"><button name>Home</button></form>"
    ;

const char *THIS_HTTP_BODY_RESTART = 
    "<div style=\"text-align:center;color:#eaeaea;\">"
    "<h3>Restart</h3>"
    "<h2>" D_DEVICE "</h2>"
    "</div>"
    "Restarting..."
    "<p></p>"
    "<form action=\"/\" method=\"get\"><button name>Home</button></form>"
    ;

const char *THIS_HTTP_BODY_END =
    "</div>"
    "</body>"
    "</html>"
    ;

void dump_wificonfig () {

    // Dump wifi
    ESP_LOGI(TAG, "wifi ap1_ssid = %s", wificonfig_vals_wifi.ap1_ssid);
    ESP_LOGI(TAG, "wifi ap1_pswd = %s", wificonfig_vals_wifi.ap1_pswd);
    ESP_LOGI(TAG, "wifi ap2_ssid = %s", wificonfig_vals_wifi.ap2_ssid);
    ESP_LOGI(TAG, "wifi ap2_pswd = %s", wificonfig_vals_wifi.ap2_pswd);
    ESP_LOGI(TAG, "wifi ap3_ssid = %s", wificonfig_vals_wifi.ap3_ssid);
    ESP_LOGI(TAG, "wifi ap3_pswd = %s", wificonfig_vals_wifi.ap3_pswd);
    ESP_LOGI(TAG, "wifi ap4_ssid = %s", wificonfig_vals_wifi.ap4_ssid);
    ESP_LOGI(TAG, "wifi ap4_pswd = %s", wificonfig_vals_wifi.ap4_pswd);
    ESP_LOGI(TAG, "wifi hostname = %s", wificonfig_vals_wifi.hostname);

    // Dump mqtt
    ESP_LOGI(TAG, "mqtt host = %s", wificonfig_vals_mqtt.host);
    ESP_LOGI(TAG, "mqtt port = %u", wificonfig_vals_mqtt.port);
    ESP_LOGI(TAG, "mqtt client = %s", wificonfig_vals_mqtt.client);
    ESP_LOGI(TAG, "mqtt user = %s", wificonfig_vals_mqtt.user);
    ESP_LOGI(TAG, "mqtt pswd = %s", wificonfig_vals_mqtt.pswd);
    ESP_LOGI(TAG, "mqtt topic = %s", wificonfig_vals_mqtt.topic);
    ESP_LOGI(TAG, "mqtt update interval = %d", wificonfig_vals_mqtt.update);

    // Dump rfid
    ESP_LOGI(TAG, "rfid acl = %s", wificonfig_vals_rfid.acl);
    ESP_LOGI(TAG, "rfid uniq = %u", wificonfig_vals_rfid.uniq);
    ESP_LOGI(TAG, "rfid host = %s", wificonfig_vals_rfid.host);
    ESP_LOGI(TAG, "rfid port = %u", wificonfig_vals_rfid.port);
    ESP_LOGI(TAG, "rfid cache = %u", wificonfig_vals_rfid.cache);
    ESP_LOGI(TAG, "rfid momentary = %u", wificonfig_vals_rfid.moment);
    ESP_LOGI(TAG, "rfid momentary timet = %u", wificonfig_vals_rfid.mtime);
}


/* An HTTP GET handler */
static esp_err_t hello_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;

    /* Get header value string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        /* Copy null terminated value string into buffer */
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Host: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-2") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-2", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-2: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-1") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-1", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-1: %s", buf);
        }
        free(buf);
    }

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[32];
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "query1", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query1=%s", param);
            }
            if (httpd_query_key_value(buf, "query3", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query3=%s", param);
            }
            if (httpd_query_key_value(buf, "query2", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query2=%s", param);
            }
        }
        free(buf);
    }

    /* Set some custom headers */
    httpd_resp_set_hdr(req, "Custom-Header-1", "Custom-Value-1");
    httpd_resp_set_hdr(req, "Custom-Header-2", "Custom-Value-2");

    /* Send response with custom headers and body set as the
     * string passed in user context*/
    const char* resp_str = (const char*) req->user_ctx;
    httpd_resp_send(req, resp_str, strlen(resp_str));

    /* After sending the HTTP response the old HTTP request
     * headers are lost. Check if HTTP request headers can be read now. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
        ESP_LOGI(TAG, "Request headers lost");
    }
    return ESP_OK;
}

static const httpd_uri_t hello = {
    .uri       = "/hello",
    .method    = HTTP_GET,
    .handler   = hello_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = "Hello World!"
};

static esp_err_t home_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "in home config handler");
    httpd_resp_send_chunk (req, THIS_HTTP_HEAD_START, strlen(THIS_HTTP_HEAD_START));
    httpd_resp_send_chunk (req, THIS_HTTP_STYLE, strlen(THIS_HTTP_STYLE));
    httpd_resp_send_chunk (req, THIS_HTTP_HEAD_END, strlen(THIS_HTTP_HEAD_END));
    httpd_resp_send_chunk (req, THIS_HTTP_BODY_START, strlen(THIS_HTTP_BODY_START));
    httpd_resp_send_chunk (req, THIS_HTTP_BODY_HOME, strlen(THIS_HTTP_BODY_HOME));
    httpd_resp_send_chunk (req, THIS_HTTP_BODY_END, strlen(THIS_HTTP_BODY_END));
    httpd_resp_send_chunk (req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t home = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = home_get_handler,
    .user_ctx  = NULL
};

void set_authmode (char* b, int authmode) {
    switch (authmode) {
    case WIFI_AUTH_OPEN:
        strcpy (b, "WIFI_AUTH_OPEN");
        break;
    case WIFI_AUTH_WEP:
        strcpy (b, "WIFI_AUTH_WEP");
        break;
    case WIFI_AUTH_WPA_PSK:
        strcpy (b, "WIFI_AUTH_WPA_PSK");
        break;
    case WIFI_AUTH_WPA2_PSK:
        strcpy (b, "WIFI_AUTH_WPA2_PSK");
        break;
    case WIFI_AUTH_WPA_WPA2_PSK:
        strcpy (b, "WIFI_AUTH_WPA_WPA2_PSK");
        break;
    case WIFI_AUTH_WPA2_ENTERPRISE:
        strcpy (b, "WIFI_AUTH_WPA2_ENTERPRISE");
        break;
    default:
        strcpy (b, "WIFI_AUTH_UNKNOWN");
        break;
    }
}

static uint16_t number = DEFAULT_SCAN_LIST_SIZE;
static wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
static uint16_t ap_count = 0;

static esp_err_t wifi_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "in wifi config handler");

    size_t buf_len;
    char* buf;

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            httpd_query_key_value(buf, "s1", wificonfig_vals_wifi.ap1_ssid, sizeof(wificonfig_vals_wifi.ap1_ssid));
            httpd_query_key_value(buf, "p1", wificonfig_vals_wifi.ap1_pswd, sizeof(wificonfig_vals_wifi.ap1_pswd));
            httpd_query_key_value(buf, "s2", wificonfig_vals_wifi.ap2_ssid, sizeof(wificonfig_vals_wifi.ap2_ssid));
            httpd_query_key_value(buf, "p2", wificonfig_vals_wifi.ap2_pswd, sizeof(wificonfig_vals_wifi.ap2_pswd));
            httpd_query_key_value(buf, "s3", wificonfig_vals_wifi.ap3_ssid, sizeof(wificonfig_vals_wifi.ap3_ssid));
            httpd_query_key_value(buf, "p3", wificonfig_vals_wifi.ap3_pswd, sizeof(wificonfig_vals_wifi.ap3_pswd));
            httpd_query_key_value(buf, "s4", wificonfig_vals_wifi.ap4_ssid, sizeof(wificonfig_vals_wifi.ap4_ssid));
            httpd_query_key_value(buf, "p4", wificonfig_vals_wifi.ap4_pswd, sizeof(wificonfig_vals_wifi.ap4_pswd));
            httpd_query_key_value(buf, "hn", wificonfig_vals_wifi.hostname, sizeof(wificonfig_vals_wifi.hostname));
        }
        free(buf);
    }

    httpd_resp_send_chunk (req, THIS_HTTP_HEAD_START, strlen(THIS_HTTP_HEAD_START));
    httpd_resp_send_chunk (req, THIS_HTTP_WIFI_SCRIPT, strlen(THIS_HTTP_WIFI_SCRIPT));
    httpd_resp_send_chunk (req, THIS_HTTP_STYLE, strlen(THIS_HTTP_STYLE));
    httpd_resp_send_chunk (req, THIS_HTTP_HEAD_END, strlen(THIS_HTTP_HEAD_END));
    httpd_resp_send_chunk (req, THIS_HTTP_BODY_START, strlen(THIS_HTTP_BODY_START));
    httpd_resp_send_chunk (req, THIS_HTTP_BODY_WIFI_0, strlen(THIS_HTTP_BODY_WIFI_0));

    if (ap_count > 0) {
        char html_buf[80];
        for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++) {
            httpd_resp_send_chunk (req, THIS_HTTP_BODY_WIFI_SCAN_1, strlen(THIS_HTTP_BODY_WIFI_SCAN_1));
            sprintf(html_buf, "%s</a>&nbsp %d &nbsp", ap_info[i].ssid, ap_info[i].rssi);
            httpd_resp_send_chunk (req, html_buf, strlen(html_buf));
            httpd_resp_send_chunk (req, THIS_HTTP_BODY_WIFI_SCAN_2, strlen(THIS_HTTP_BODY_WIFI_SCAN_2));
            set_authmode (html_buf, ap_info[i].authmode);
            httpd_resp_send_chunk (req, html_buf, strlen(html_buf));
            httpd_resp_send_chunk (req, THIS_HTTP_BODY_WIFI_SCAN_3, strlen(THIS_HTTP_BODY_WIFI_SCAN_3));
        }
        
    } else {
        httpd_resp_send_chunk (req, THIS_HTTP_BODY_WIFI_SCAN_0, strlen(THIS_HTTP_BODY_WIFI_SCAN_0));
    }

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_WIFI_1, strlen(THIS_HTTP_BODY_WIFI_1));
    if (strlen(wificonfig_vals_wifi.ap1_ssid) > 0)
        httpd_resp_send_chunk (req, wificonfig_vals_wifi.ap1_ssid, strlen(wificonfig_vals_wifi.ap1_ssid));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_WIFI_2, strlen(THIS_HTTP_BODY_WIFI_2));
    if (strlen(wificonfig_vals_wifi.ap1_pswd) > 0)
        httpd_resp_send_chunk (req, wificonfig_vals_wifi.ap1_pswd, strlen(wificonfig_vals_wifi.ap1_pswd));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_WIFI_3, strlen(THIS_HTTP_BODY_WIFI_3));
    if (strlen(wificonfig_vals_wifi.ap2_ssid) > 0)
        httpd_resp_send_chunk (req, wificonfig_vals_wifi.ap2_ssid, strlen(wificonfig_vals_wifi.ap2_ssid));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_WIFI_4, strlen(THIS_HTTP_BODY_WIFI_4));
    if (strlen(wificonfig_vals_wifi.ap2_pswd) > 0)
        httpd_resp_send_chunk (req, wificonfig_vals_wifi.ap2_pswd, strlen(wificonfig_vals_wifi.ap2_pswd));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_WIFI_5, strlen(THIS_HTTP_BODY_WIFI_5));
    if (strlen(wificonfig_vals_wifi.ap3_ssid) > 0)
        httpd_resp_send_chunk (req, wificonfig_vals_wifi.ap3_ssid, strlen(wificonfig_vals_wifi.ap3_ssid));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_WIFI_6, strlen(THIS_HTTP_BODY_WIFI_6));
    if (strlen(wificonfig_vals_wifi.ap3_pswd) > 0)
        httpd_resp_send_chunk (req, wificonfig_vals_wifi.ap3_pswd, strlen(wificonfig_vals_wifi.ap3_pswd));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_WIFI_7, strlen(THIS_HTTP_BODY_WIFI_7));
    if (strlen(wificonfig_vals_wifi.ap4_ssid) > 0)
        httpd_resp_send_chunk (req, wificonfig_vals_wifi.ap4_ssid, strlen(wificonfig_vals_wifi.ap4_ssid));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_WIFI_8, strlen(THIS_HTTP_BODY_WIFI_8));
    if (strlen(wificonfig_vals_wifi.ap4_pswd) > 0)
        httpd_resp_send_chunk (req, wificonfig_vals_wifi.ap4_pswd, strlen(wificonfig_vals_wifi.ap4_pswd));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_WIFI_9, strlen(THIS_HTTP_BODY_WIFI_9));
    if (strlen(wificonfig_vals_wifi.hostname) > 0)
        httpd_resp_send_chunk (req, wificonfig_vals_wifi.hostname, strlen(wificonfig_vals_wifi.hostname));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_WIFI_10, strlen(THIS_HTTP_BODY_WIFI_10));
    httpd_resp_send_chunk (req, THIS_HTTP_BODY_END, strlen(THIS_HTTP_BODY_END));
    httpd_resp_send_chunk (req, NULL, 0);


    return ESP_OK;
}

static const httpd_uri_t wifi = {
    .uri       = "/wifi",
    .method    = HTTP_GET,
    .handler   = wifi_get_handler,
    .user_ctx  = NULL
};

void validate_host (char *val, char *host) {
    unsigned int byte[4];
    if (strcmp (val, "") == 0) {
        // need to disable MQTT
        strcpy (host, "");
        return;
    }
    if (sscanf (val, "%u.%u.%u.%u", &(byte[0]), &(byte[1]), &(byte[2]), &(byte[3])) == 4) {
        if ((byte[0] <= 255) &&
            (byte[1] <= 255) &&
            (byte[2] <= 255) &&
            (byte[3] <= 255)) {
            sprintf (host, "%u.%u.%u.%u", byte[0], byte[1], byte[2], byte[3]);
        }
    }
}

void validate_u16 (char *val, unsigned int min, unsigned int max, uint16_t *number) {
    unsigned int sp;
    if (sscanf (val, "%u", &sp) == 1) {
        if ((sp >= min) && (sp <= max)) {
            *number = (uint16_t) sp;
        }
    }
}

void validate_u8 (char *val, unsigned int min, unsigned int max, uint8_t *number) {
    unsigned int sp;
    if (sscanf (val, "%u", &sp) == 1) {
        if ((sp >= min) && (sp <= max)) {
            *number = (uint16_t) sp;
        }
    }
}

static esp_err_t mqtt_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "in mqtt config handler");

    char val[20];
    char val_str[10];
    size_t buf_len;
    char* buf;

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            httpd_query_key_value(buf, "ho", val, sizeof(val));
            validate_host (val, wificonfig_vals_mqtt.host);
            httpd_query_key_value(buf, "po", val, sizeof(val));
            validate_u16 (val, 1, 65535, &wificonfig_vals_mqtt.port);
            httpd_query_key_value(buf, "cl", wificonfig_vals_mqtt.client, sizeof(wificonfig_vals_mqtt.client));
            httpd_query_key_value(buf, "us", wificonfig_vals_mqtt.user, sizeof(wificonfig_vals_mqtt.user));
            httpd_query_key_value(buf, "pa", wificonfig_vals_mqtt.pswd, sizeof(wificonfig_vals_mqtt.pswd));
            httpd_query_key_value(buf, "to", wificonfig_vals_mqtt.topic, sizeof(wificonfig_vals_mqtt.topic));
            httpd_query_key_value(buf, "up", val, sizeof(val));
            validate_u16 (val, 0, 240, &wificonfig_vals_mqtt.update);
        }
        free(buf);
    }


    httpd_resp_send_chunk (req, THIS_HTTP_HEAD_START, strlen(THIS_HTTP_HEAD_START));
    httpd_resp_send_chunk (req, THIS_HTTP_STYLE, strlen(THIS_HTTP_STYLE));
    httpd_resp_send_chunk (req, THIS_HTTP_HEAD_END, strlen(THIS_HTTP_HEAD_END));
    httpd_resp_send_chunk (req, THIS_HTTP_BODY_START, strlen(THIS_HTTP_BODY_START));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_MQTT_0, strlen(THIS_HTTP_BODY_MQTT_0));
    httpd_resp_send_chunk (req, THIS_HTTP_BODY_MQTT_1, strlen(THIS_HTTP_BODY_MQTT_1));
    if (strlen(wificonfig_vals_mqtt.host) > 0)
        httpd_resp_send_chunk (req, wificonfig_vals_mqtt.host, strlen(wificonfig_vals_mqtt.host));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_MQTT_2, strlen(THIS_HTTP_BODY_MQTT_2));
    sprintf (val_str, "%u", wificonfig_vals_mqtt.port);
    httpd_resp_send_chunk (req, val_str, strlen(val_str));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_MQTT_3, strlen(THIS_HTTP_BODY_MQTT_3));
    if (strlen(wificonfig_vals_mqtt.client) > 0)
        httpd_resp_send_chunk (req, wificonfig_vals_mqtt.client, strlen(wificonfig_vals_mqtt.client));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_MQTT_4, strlen(THIS_HTTP_BODY_MQTT_4));
    if (strlen(wificonfig_vals_mqtt.user) > 0)
        httpd_resp_send_chunk (req, wificonfig_vals_mqtt.user, strlen(wificonfig_vals_mqtt.user));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_MQTT_5, strlen(THIS_HTTP_BODY_MQTT_5));
    if (strlen(wificonfig_vals_mqtt.pswd) > 0)
        httpd_resp_send_chunk (req, wificonfig_vals_mqtt.pswd, strlen(wificonfig_vals_mqtt.pswd));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_MQTT_6, strlen(THIS_HTTP_BODY_MQTT_6));
    if (strlen(wificonfig_vals_mqtt.topic) > 0)
        httpd_resp_send_chunk (req, wificonfig_vals_mqtt.topic, strlen(wificonfig_vals_mqtt.topic));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_MQTT_7, strlen(THIS_HTTP_BODY_MQTT_7));
    sprintf (val_str, "%u", wificonfig_vals_mqtt.update);
    httpd_resp_send_chunk (req, val_str, strlen(val_str));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_MQTT_8, strlen(THIS_HTTP_BODY_MQTT_8));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_END, strlen(THIS_HTTP_BODY_END));
    httpd_resp_send_chunk (req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t mqtt = {
    .uri       = "/mqtt",
    .method    = HTTP_GET,
    .handler   = mqtt_get_handler,
    .user_ctx  = NULL
};

static esp_err_t rfid_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "in rfid config handler");

    char val[20];
    char num_str[10];
    size_t buf_len;
    char* buf;

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            httpd_query_key_value(buf, "ac", wificonfig_vals_rfid.acl, sizeof(wificonfig_vals_rfid.acl));
            httpd_query_key_value(buf, "un", val, sizeof(val));
            validate_u8 (val, 0, 255, &wificonfig_vals_rfid.uniq);
            httpd_query_key_value(buf, "ho", val, sizeof(val));
            validate_host (val, wificonfig_vals_rfid.host);
            httpd_query_key_value(buf, "po", val, sizeof(val));
            validate_u16 (val, 1, 65535, &wificonfig_vals_rfid.port);
            httpd_query_key_value(buf, "ca", val, sizeof(val));
            validate_u8 (val, 0, 1, &wificonfig_vals_rfid.cache);
            httpd_query_key_value(buf, "mo", val, sizeof(val));
            validate_u8 (val, 0, 1, &wificonfig_vals_rfid.moment);
            httpd_query_key_value(buf, "mt", val, sizeof(val));
            validate_u16 (val, 1, 600, &wificonfig_vals_rfid.mtime);
        }
        free(buf);
    }


    httpd_resp_send_chunk (req, THIS_HTTP_HEAD_START, strlen(THIS_HTTP_HEAD_START));
    httpd_resp_send_chunk (req, THIS_HTTP_STYLE, strlen(THIS_HTTP_STYLE));
    httpd_resp_send_chunk (req, THIS_HTTP_HEAD_END, strlen(THIS_HTTP_HEAD_END));
    httpd_resp_send_chunk (req, THIS_HTTP_BODY_START, strlen(THIS_HTTP_BODY_START));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_RFID_0, strlen(THIS_HTTP_BODY_RFID_0));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_RFID_1, strlen(THIS_HTTP_BODY_RFID_1));
    httpd_resp_send_chunk (req, wificonfig_vals_rfid.acl, strlen(wificonfig_vals_rfid.acl));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_RFID_2, strlen(THIS_HTTP_BODY_RFID_2));
    sprintf (num_str, "%u", wificonfig_vals_rfid.uniq);
    httpd_resp_send_chunk (req, num_str, strlen(num_str));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_RFID_3, strlen(THIS_HTTP_BODY_RFID_3));
    httpd_resp_send_chunk (req, wificonfig_vals_rfid.host, strlen(wificonfig_vals_rfid.host));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_RFID_4, strlen(THIS_HTTP_BODY_RFID_4));
    sprintf (num_str, "%u", wificonfig_vals_rfid.port);
    httpd_resp_send_chunk (req, num_str, strlen(num_str));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_RFID_5, strlen(THIS_HTTP_BODY_RFID_5));
    sprintf (num_str, "%u", wificonfig_vals_rfid.cache);
    httpd_resp_send_chunk (req, num_str, strlen(num_str));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_RFID_6, strlen(THIS_HTTP_BODY_RFID_6));
    sprintf (num_str, "%u", wificonfig_vals_rfid.moment);
    httpd_resp_send_chunk (req, num_str, strlen(num_str));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_RFID_7, strlen(THIS_HTTP_BODY_RFID_7));
    sprintf (num_str, "%u", wificonfig_vals_rfid.mtime);
    httpd_resp_send_chunk (req, num_str, strlen(num_str));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_RFID_8, strlen(THIS_HTTP_BODY_RFID_8));

    httpd_resp_send_chunk (req, THIS_HTTP_BODY_END, strlen(THIS_HTTP_BODY_END));
    httpd_resp_send_chunk (req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t rfid = {
    .uri       = "/rfid",
    .method    = HTTP_GET,
    .handler   = rfid_get_handler,
    .user_ctx  = NULL
};

static esp_err_t save_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "in save config handler");

    esp_err_t err;

    // try to save values to NVS
    nvs_handle_t my_handle;
    err = nvs_open(D_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        char *resp_str = "Error opening NVS handle!";
        httpd_resp_send(req, resp_str, strlen(resp_str));
        return (err);
    } else {
        int8_t nvs_valid = true;
        err = nvs_set_i8 (my_handle, "valid_flag", nvs_valid);
        if (err != ESP_OK) {
            char *resp_str = "Error setting valid_flag in NVS!";
            httpd_resp_send(req, resp_str, strlen(resp_str));
            nvs_close (my_handle);
            return (err);
        }

        // save wifi configuration to NVS
        if (((err = nvs_set_str (my_handle, "wifi_ap1_ssid", wificonfig_vals_wifi.ap1_ssid)) != ESP_OK) ||
            ((err = nvs_set_str (my_handle, "wifi_ap1_pswd", wificonfig_vals_wifi.ap1_pswd)) != ESP_OK) ||
            ((err = nvs_set_str (my_handle, "wifi_ap2_ssid", wificonfig_vals_wifi.ap2_ssid)) != ESP_OK) ||
            ((err = nvs_set_str (my_handle, "wifi_ap2_pswd", wificonfig_vals_wifi.ap2_pswd)) != ESP_OK) ||
            ((err = nvs_set_str (my_handle, "wifi_ap3_ssid", wificonfig_vals_wifi.ap3_ssid)) != ESP_OK) ||
            ((err = nvs_set_str (my_handle, "wifi_ap3_pswd", wificonfig_vals_wifi.ap3_pswd)) != ESP_OK) ||
            ((err = nvs_set_str (my_handle, "wifi_ap4_ssid", wificonfig_vals_wifi.ap4_ssid)) != ESP_OK) ||
            ((err = nvs_set_str (my_handle, "wifi_ap4_pswd", wificonfig_vals_wifi.ap4_pswd)) != ESP_OK) ||
            ((err = nvs_set_str (my_handle, "wifi_hostname", wificonfig_vals_wifi.hostname)) != ESP_OK)) {

            char *resp_str = "Error setting wifi values in NVS!";
            httpd_resp_send(req, resp_str, strlen(resp_str));
            nvs_close (my_handle);
            return (err);
        }

        // save wifi configuration to NVS
        if (((err = nvs_set_str (my_handle, "mqtt_host",   wificonfig_vals_mqtt.host))   != ESP_OK) ||
            ((err = nvs_set_u16 (my_handle, "mqtt_port",   wificonfig_vals_mqtt.port))   != ESP_OK) ||
            ((err = nvs_set_str (my_handle, "mqtt_client", wificonfig_vals_mqtt.client)) != ESP_OK) ||
            ((err = nvs_set_str (my_handle, "mqtt_user",   wificonfig_vals_mqtt.user))   != ESP_OK) ||
            ((err = nvs_set_str (my_handle, "mqtt_pswd",   wificonfig_vals_mqtt.pswd))   != ESP_OK) ||
            ((err = nvs_set_str (my_handle, "mqtt_topic",  wificonfig_vals_mqtt.topic))  != ESP_OK) ||
            ((err = nvs_set_u16 (my_handle, "mqtt_update", wificonfig_vals_mqtt.update)) != ESP_OK)) {

            char *resp_str = "Error setting mqtt values in NVS!";
            httpd_resp_send(req, resp_str, strlen(resp_str));
            nvs_close (my_handle);
            return (err);
        }


        // save rfid configuration to NVS
        if (((err = nvs_set_str (my_handle, "rfid_acl",    wificonfig_vals_rfid.acl))    != ESP_OK) ||
            ((err = nvs_set_u8  (my_handle, "rfid_uniq",   wificonfig_vals_rfid.uniq))   != ESP_OK) ||
            ((err = nvs_set_str (my_handle, "rfid_host",   wificonfig_vals_rfid.host))   != ESP_OK) ||
            ((err = nvs_set_u16 (my_handle, "rfid_port",   wificonfig_vals_rfid.port))   != ESP_OK) ||
            ((err = nvs_set_u8  (my_handle, "rfid_cache",  wificonfig_vals_rfid.cache))  != ESP_OK) ||
            ((err = nvs_set_u8  (my_handle, "rfid_moment", wificonfig_vals_rfid.moment)) != ESP_OK) ||
            ((err = nvs_set_u16 (my_handle, "rfid_mtime",  wificonfig_vals_rfid.mtime))  != ESP_OK)) {

            char *resp_str = "Error setting rfid values in NVS!";
            httpd_resp_send(req, resp_str, strlen(resp_str));
            nvs_close (my_handle);
            return (err);
        }
        
        err = nvs_commit (my_handle);
        if (err != ESP_OK) {
            char *resp_str = "Error committing NVS!";
            httpd_resp_send(req, resp_str, strlen(resp_str));
            nvs_close (my_handle);
            return (err);
        }

        nvs_close (my_handle);
    }

    httpd_resp_send_chunk (req, THIS_HTTP_HEAD_START, strlen(THIS_HTTP_HEAD_START));
    httpd_resp_send_chunk (req, THIS_HTTP_STYLE, strlen(THIS_HTTP_STYLE));
    httpd_resp_send_chunk (req, THIS_HTTP_HEAD_END, strlen(THIS_HTTP_HEAD_END));
    httpd_resp_send_chunk (req, THIS_HTTP_BODY_START, strlen(THIS_HTTP_BODY_START));
    httpd_resp_send_chunk (req, THIS_HTTP_BODY_SAVE, strlen(THIS_HTTP_BODY_SAVE));
    httpd_resp_send_chunk (req, THIS_HTTP_BODY_END, strlen(THIS_HTTP_BODY_END));
    httpd_resp_send_chunk (req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t save = {
    .uri       = "/save",
    .method    = HTTP_GET,
    .handler   = save_get_handler,
    .user_ctx  = NULL
};


static esp_err_t restart_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "in restart config handler");

    httpd_resp_send_chunk (req, THIS_HTTP_HEAD_START, strlen(THIS_HTTP_HEAD_START));
    httpd_resp_send_chunk (req, THIS_HTTP_STYLE, strlen(THIS_HTTP_STYLE));
    httpd_resp_send_chunk (req, THIS_HTTP_HEAD_END, strlen(THIS_HTTP_HEAD_END));
    httpd_resp_send_chunk (req, THIS_HTTP_BODY_START, strlen(THIS_HTTP_BODY_START));
    httpd_resp_send_chunk (req, THIS_HTTP_BODY_RESTART, strlen(THIS_HTTP_BODY_RESTART));
    httpd_resp_send_chunk (req, THIS_HTTP_BODY_END, strlen(THIS_HTTP_BODY_END));
    httpd_resp_send_chunk (req, NULL, 0);

    // wait two seconds before resetting so page can be served up
    vTaskDelay(2000 / portTICK_RATE_MS);

    esp_restart();
}

static const httpd_uri_t restart = {
    .uri       = "/restart",
    .method    = HTTP_GET,
    .handler   = restart_get_handler,
    .user_ctx  = NULL
};


/* This handler allows the custom error handling functionality to be
 * tested from client side. For that, when a PUT request 0 is sent to
 * URI /ctrl, the /hello and /echo URIs are unregistered and following
 * custom error handler http_404_error_handler() is registered.
 * Afterwards, when /hello or /echo is requested, this custom error
 * handler is invoked which, after sending an error message to client,
 * either closes the underlying socket (when requested URI is /echo)
 * or keeps it open (when requested URI is /hello). This allows the
 * client to infer if the custom error handler is functioning as expected
 * by observing the socket state.
 */
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    if (strcmp("/hello", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/hello URI is not available");
        /* Return ESP_OK to keep underlying socket open */
        return ESP_OK;
    } else if (strcmp("/echo", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/echo URI is not available");
        /* Return ESP_FAIL to close underlying socket */
        return ESP_FAIL;
    }
    /* For any other URI send 404 and close socket */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &hello);
        httpd_register_uri_handler(server, &home);
        httpd_register_uri_handler(server, &wifi);
        httpd_register_uri_handler(server, &mqtt);
        httpd_register_uri_handler(server, &rfid);
        httpd_register_uri_handler(server, &save);
        httpd_register_uri_handler(server, &restart);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "ESP32 is started in AP mode");

    } else if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void wificonfig_scan(void) {
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    memset(ap_info, 0, sizeof(ap_info));

    ap_count = 0;
    ESP_LOGI(TAG, "Starting AP scan");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, true));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    ESP_ERROR_CHECK(esp_wifi_stop());
}

void wifi_init_ap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wificonfig_scan();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s",
             WIFI_SSID, WIFI_PASS);
}

// set configuration values to defaults
//
void init_wificonfig(void) {
    uint8_t mac[6];
    esp_base_mac_addr_get(mac);
    uint16_t id = mac[5] + (mac[4]<<8);

    // wifi
    strcpy (wificonfig_vals_wifi.ap1_ssid, "");
    strcpy (wificonfig_vals_wifi.ap1_pswd, "");
    strcpy (wificonfig_vals_wifi.ap2_ssid, "");
    strcpy (wificonfig_vals_wifi.ap2_pswd, "");
    strcpy (wificonfig_vals_wifi.ap3_ssid, "");
    strcpy (wificonfig_vals_wifi.ap3_pswd, "");
    strcpy (wificonfig_vals_wifi.ap4_ssid, "");
    strcpy (wificonfig_vals_wifi.ap4_pswd, "");
    sprintf (wificonfig_vals_wifi.hostname, "%s-%d", D_DEVICE, id);

    // wifi
    strcpy (wificonfig_vals_mqtt.host, "");
    wificonfig_vals_mqtt.port = 1883;
    sprintf (wificonfig_vals_mqtt.client, "%s-%d", D_DEVICE, id);
    strcpy (wificonfig_vals_mqtt.user, "");
    strcpy (wificonfig_vals_mqtt.pswd, "");
    strcpy (wificonfig_vals_mqtt.topic, D_DEVICE);
    wificonfig_vals_mqtt.update = 5;

    // rfid
    strcpy (wificonfig_vals_rfid.acl, "door");
    wificonfig_vals_rfid.uniq = 0;
    strcpy (wificonfig_vals_rfid.host, "");
    wificonfig_vals_rfid.port = 8080;
    wificonfig_vals_rfid.cache = 0;
    wificonfig_vals_rfid.moment = 1;
    wificonfig_vals_rfid.mtime = 5;

}
// read configuration values from NVS
//
esp_err_t get_nvs_wificonfig(nvs_handle_t my_handle) {
    esp_err_t err;
    esp_err_t last_err = ESP_OK;
    size_t ss;

    // wifi parameters
    if ((err = nvs_get_str(my_handle, "wifi_ap1_ssid", NULL,                          &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_str(my_handle, "wifi_ap1_ssid", wificonfig_vals_wifi.ap1_ssid, &ss)) != ESP_OK) last_err = err;

    if ((err = nvs_get_str(my_handle, "wifi_ap1_pswd", NULL,                          &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_str(my_handle, "wifi_ap1_pswd", wificonfig_vals_wifi.ap1_pswd, &ss)) != ESP_OK) last_err = err;

    if ((err = nvs_get_str(my_handle, "wifi_ap2_ssid", NULL,                          &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_str(my_handle, "wifi_ap2_ssid", wificonfig_vals_wifi.ap2_ssid, &ss)) != ESP_OK) last_err = err;

    if ((err = nvs_get_str(my_handle, "wifi_ap2_pswd", NULL,                          &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_str(my_handle, "wifi_ap2_pswd", wificonfig_vals_wifi.ap2_pswd, &ss)) != ESP_OK) last_err = err;

    if ((err = nvs_get_str(my_handle, "wifi_ap3_ssid", NULL,                          &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_str(my_handle, "wifi_ap3_ssid", wificonfig_vals_wifi.ap3_ssid, &ss)) != ESP_OK) last_err = err;

    if ((err = nvs_get_str(my_handle, "wifi_ap3_pswd", NULL,                          &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_str(my_handle, "wifi_ap3_pswd", wificonfig_vals_wifi.ap3_pswd, &ss)) != ESP_OK) last_err = err;

    if ((err = nvs_get_str(my_handle, "wifi_ap4_ssid", NULL,                          &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_str(my_handle, "wifi_ap4_ssid", wificonfig_vals_wifi.ap4_ssid, &ss)) != ESP_OK) last_err = err;

    if ((err = nvs_get_str(my_handle, "wifi_ap4_pswd", NULL,                          &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_str(my_handle, "wifi_ap4_pswd", wificonfig_vals_wifi.ap4_pswd, &ss)) != ESP_OK) last_err = err;

    if ((err = nvs_get_str(my_handle, "wifi_hostname", NULL,                          &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_str(my_handle, "wifi_hostname", wificonfig_vals_wifi.hostname, &ss)) != ESP_OK) last_err = err;

    // mqtt parameters
    if ((err = nvs_get_str(my_handle, "mqtt_host", NULL,                      &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_str(my_handle, "mqtt_host", wificonfig_vals_mqtt.host, &ss)) != ESP_OK) last_err = err;

    if ((err = nvs_get_u16(my_handle, "mqtt_port", &wificonfig_vals_mqtt.port)) != ESP_OK) last_err = err;

    if ((err = nvs_get_str(my_handle, "mqtt_client", NULL,                        &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_str(my_handle, "mqtt_client", wificonfig_vals_mqtt.client, &ss)) != ESP_OK) last_err = err;

    if ((err = nvs_get_str(my_handle, "mqtt_user", NULL,                      &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_str(my_handle, "mqtt_user", wificonfig_vals_mqtt.user, &ss)) != ESP_OK) last_err = err;

    if ((err = nvs_get_str(my_handle, "mqtt_pswd", NULL,                      &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_str(my_handle, "mqtt_pswd", wificonfig_vals_mqtt.pswd, &ss)) != ESP_OK) last_err = err;

    if ((err = nvs_get_str(my_handle, "mqtt_topic", NULL,                       &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_str(my_handle, "mqtt_topic", wificonfig_vals_mqtt.topic, &ss)) != ESP_OK) last_err = err;

    if ((err = nvs_get_u16(my_handle, "mqtt_update", &wificonfig_vals_mqtt.update)) != ESP_OK) last_err = err;

    // rfid parameters
    if ((err = nvs_get_str(my_handle, "rfid_acl", NULL,                     &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_str(my_handle, "rfid_acl", wificonfig_vals_rfid.acl, &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_u8(my_handle,  "rfid_uniq", &wificonfig_vals_rfid.uniq)) != ESP_OK) last_err = err;
    if ((err = nvs_get_str(my_handle, "rfid_host", NULL,                      &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_str(my_handle, "rfid_host", wificonfig_vals_rfid.host, &ss)) != ESP_OK) last_err = err;
    if ((err = nvs_get_u16(my_handle, "rfid_port", &wificonfig_vals_rfid.port)) != ESP_OK) last_err = err;
    if ((err = nvs_get_u8(my_handle,  "rfid_cache", &wificonfig_vals_rfid.cache)) != ESP_OK) last_err = err;
    if ((err = nvs_get_u8(my_handle,  "rfid_moment", &wificonfig_vals_rfid.moment)) != ESP_OK) last_err = err;
    if ((err = nvs_get_u16(my_handle,  "rfid_mtime", &wificonfig_vals_rfid.mtime)) != ESP_OK) last_err = err;

    return (last_err);
}

void trigger_wificonfig () {
    ESP_LOGI(TAG, "in trigger_wificonifg");

    esp_err_t err;

    // try to save values to NVS
    nvs_handle_t my_handle;
    err = nvs_open(D_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE (TAG, "Error opening NVS handle!");
    } else {
        int8_t nvs_valid = false;
        err = nvs_set_i8 (my_handle, "valid_flag", nvs_valid);
        if (err != ESP_OK) {
            ESP_LOGE (TAG, "Error clearing valid_flag in NVS!");
            nvs_close (my_handle);
            return;
        }
        
        err = nvs_commit (my_handle);
        if (err != ESP_OK) {
            ESP_LOGE (TAG, "Error committing NVS!");
            nvs_close (my_handle);
            return;
        }

        nvs_close (my_handle);
        ESP_LOGI (TAG, "Restarting...");
        esp_restart();

    }

}

esp_err_t wificonfig(void)
{

    // Initialize NVS
    esp_err_t err = nvs_flash_init ();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_LOGI(TAG, "Erasing NVS flash");
        ESP_ERROR_CHECK(nvs_flash_erase ());
        err = nvs_flash_init ();
    }
    ESP_ERROR_CHECK (err);
    ESP_LOGI(TAG, "Done with NVS init");

    init_wificonfig ();

    // try to read values from NVS
    nvs_handle_t my_handle;
    int8_t nvs_valid = false;
    err = nvs_open(D_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return (err);
    } else {
        ESP_LOGI(TAG, "Opened NVS");
        err = nvs_get_i8 (my_handle, "valid_flag", &nvs_valid);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Read NVS valid_flag = %d", nvs_valid);
            if ((get_nvs_wificonfig(my_handle) == ESP_OK) && nvs_valid) {
                ESP_LOGI(TAG, "Have valid config data, returning to main");
                nvs_close (my_handle);
                dump_wificonfig();
                return (ESP_OK);
            }
        } else {
            nvs_valid = false;
        }

        ESP_LOGI(TAG, "invalid config data, starting server");
        nvs_close (my_handle);
    }
    dump_wificonfig();

    // if we are here, NVS isn't valid -> fire up the AP and webserver
    //

    /* Start the access-point */
    wifi_init_ap();

    start_webserver();

    // wait forever until restart
    while (1) {
        vTaskDelay(100 / portTICK_RATE_MS);
    }
    return (ESP_OK);
}
