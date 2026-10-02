package com.example.myapplication;
import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import android.annotation.SuppressLint;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.os.Looper;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;
import org.json.JSONException;
import org.json.JSONObject;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
public class MQTT_ALYActivity extends AppCompatActivity {
    private TextView tv_temp;
    private TextView tv_humi;
    private TextView tv_Water_Flow;
    private TextView tV_Pressure;
    private TextView tv_smoke_density;
    private MqttClient client;
    private MqttConnectOptions options;
    private Handler handler;
    private final String sub_topic = "/sys/k1tjfOebU45/APP_dev/thing/service/property/set";//订阅
    private double temperature = 1;
    private double humidity = 1;
    private double Water_Flow = 0;
    private double LightLux = 0;
    private double ultrasound_distance = 0;
//    private double Pressure = 0;
    private TextView tv_roll, tv_pitch, tv_yaw;

    ////    private int LightLux = 0;
    ////    private int LED_1 = 1;
    ////    private double  smoke_density = 0 ;
    private boolean moveOnState = true;
    private boolean backOnState = true;
    private boolean leftOnState = true;
    private boolean rightOnState = true;
    @SuppressLint("HandlerLeak")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_mqtt_alyactivity);
        tv_temp = findViewById(R.id.textview_mqtt_interface_temp_show);
        tv_humi = findViewById(R.id.textview_mqtt_interface_hum_show);
//        tv_Water_Flow = findViewById(R.id.textview_mqtt_interface_Water_Flow_show);
//        tV_Pressure   = findViewById(R.id.textview_mqtt_interface_Pressure_show);
        tv_roll = findViewById(R.id.textview_mqtt_interface_roll_show);
        tv_pitch = findViewById(R.id.textview_mqtt_interface_pitch_show);
        tv_yaw = findViewById(R.id.textview_mqtt_interface_yaw_show);

        TextView tv_LightLux = findViewById(R.id.textview_mqtt_interface_Light_intensity_show);
        TextView tv_ultrasound_distance = findViewById(R.id.textview_mqtt_interface_ultrasound_distance_show);
        tv_smoke_density = findViewById(R.id.textview_mqtt_interface_smoke_density_show);
        Button btn_open = findViewById(R.id.button_mqtt_interface_button_on);
        Button btn_close = findViewById(R.id.button_mqtt_interface_button_off);
        Button button_move = findViewById(R.id.button_mqtt_interface_forward);
        Button button_back = findViewById(R.id.button_mqtt_interface_back);
        Button button_left_rotate = findViewById(R.id.button_mqtt_interface_rotate_left);
        Button button_right_rotate = findViewById(R.id.button_mqtt_interface_rotate_right);
        Button button_left = findViewById(R.id.button_mqtt_interface_left);
        Button button_right = findViewById(R.id.button_mqtt_interface_right);
        Button button_stop = findViewById(R.id.button_mqtt_interface_stop);
//        btn_open.setOnClickListener(v -> {
//            publish_message("{\"params\":{\"LED_1\":1},\"version\":\"1.0.0\"}");
//            Toast.makeText(MQTT_ALYActivity.this, "LED打开", Toast.LENGTH_SHORT).show();
//        });
//        btn_close.setOnClickListener(v -> {
//            publish_message("{\"params\":{\"LED_1\":0},\"version\":\"1.0.0\"}");
//            Toast.makeText(MQTT_ALYActivity.this, "LED关闭", Toast.LENGTH_SHORT).show();
//        }
        btn_open.setOnClickListener(v -> {
            // 将 LED_1 替换为 Euler_angle_open
            publish_message("{\"params\":{\"Euler_angle_open\":1},\"version\":\"1.0.0\"}");
            Toast.makeText(MQTT_ALYActivity.this, "已下发开启欧拉角指令", Toast.LENGTH_SHORT).show();
        });

        btn_close.setOnClickListener(v -> {
            // 将 LED_1 替换为 Euler_angle_open
            publish_message("{\"params\":{\"Euler_angle_open\":0},\"version\":\"1.0.0\"}");
            Toast.makeText(MQTT_ALYActivity.this, "已下发关闭欧拉角指令", Toast.LENGTH_SHORT).show();
        });
        button_move.setOnClickListener(v -> {
            if (moveOnState) {
                publish_message("{\"params\":{\"move_on\":1},\"version\":\"1.0.0\"}");
                Toast.makeText(MQTT_ALYActivity.this, "机器人前进", Toast.LENGTH_SHORT).show();
            } else {
                publish_message("{\"params\":{\"move_on\":0},\"version\":\"1.0.0\"}");
                Toast.makeText(MQTT_ALYActivity.this, "机器人前进停止", Toast.LENGTH_SHORT).show();
            }
            moveOnState = !moveOnState;
        });
        button_back.setOnClickListener(v -> {
            if (backOnState) {
                publish_message("{\"params\":{\"move_back\":1},\"version\":\"1.0.0\"}");
                Toast.makeText(MQTT_ALYActivity.this, "机器人后退", Toast.LENGTH_SHORT).show();
            } else {
                publish_message("{\"params\":{\"move_back\":0},\"version\":\"1.0.0\"}");
                Toast.makeText(MQTT_ALYActivity.this, "机器人后退停止", Toast.LENGTH_SHORT).show();
            }
            backOnState = !backOnState;
        });
        button_stop.setOnClickListener(v -> {
            if (backOnState) {
                publish_message("{\"params\":{\"move_stop\":1},\"version\":\"1.0.0\"}");
                Toast.makeText(MQTT_ALYActivity.this, "机器人停止", Toast.LENGTH_SHORT).show();
            } else {
                publish_message("{\"params\":{\"move_stop\":0},\"version\":\"1.0.0\"}");
                Toast.makeText(MQTT_ALYActivity.this, "机器人保持原始运动状态", Toast.LENGTH_SHORT).show();
            }
            backOnState = !backOnState;
        });
        button_left.setOnClickListener(v -> {
            if (leftOnState) {
                publish_message("{\"params\":{\"move_left\":1},\"version\":\"1.0.0\"}");
                Toast.makeText(MQTT_ALYActivity.this, "机器人左转", Toast.LENGTH_SHORT).show();
            }
            else {
                publish_message("{\"params\":{\"move_left\":0},\"version\":\"1.0.0\"}");
                Toast.makeText(MQTT_ALYActivity.this, "机器人左转停止", Toast.LENGTH_SHORT).show();
            }
            leftOnState = !leftOnState;
        });
        button_right.setOnClickListener(v -> {
            if (rightOnState) {
                publish_message("{\"params\":{\"move_right\":1},\"version\":\"1.0.0\"}");
                Toast.makeText(MQTT_ALYActivity.this, "机器人右转", Toast.LENGTH_SHORT).show();
            } else {
                publish_message("{\"params\":{\"move_right\":0},\"version\":\"1.0.0\"}");
                Toast.makeText(MQTT_ALYActivity.this, "机器人右转停止", Toast.LENGTH_SHORT).show();
            }
            rightOnState = !rightOnState;
        });
        button_left_rotate.setOnClickListener(v -> {
            if (leftOnState) {
                publish_message("{\"params\":{\"move_left_rotate\":1},\"version\":\"1.0.0\"}");
                Toast.makeText(MQTT_ALYActivity.this, "机器人左旋转", Toast.LENGTH_SHORT).show();
            }
            else {
                publish_message("{\"params\":{\"move_left_rotate\":0},\"version\":\"1.0.0\"}");
                Toast.makeText(MQTT_ALYActivity.this, "机器人左旋转停止", Toast.LENGTH_SHORT).show();
            }
            leftOnState = !leftOnState;
        });
        button_right_rotate.setOnClickListener(v -> {
            if (rightOnState) {
                publish_message("{\"params\":{\"move_right_rotate\":1},\"version\":\"1.0.0\"}");
                Toast.makeText(MQTT_ALYActivity.this, "机器人右旋转", Toast.LENGTH_SHORT).show();
            } else {
                publish_message("{\"params\":{\"move_right_rotate\":0},\"version\":\"1.0.0\"}");
                Toast.makeText(MQTT_ALYActivity.this, "机器人右旋转停止", Toast.LENGTH_SHORT).show();
            }
            rightOnState = !rightOnState;
        });

//        btn_close.setOnClickListener(new View.OnClickListener() {
//            @Override
//            public void onClick(View v) {
//                //关
//                showToast("LED 关", 500); // 显示500毫秒
//               //publish_message("{\"params\":{\"led\":off}}");
//            }
//        });

        mqtt_init();
        start_reconnect();
        handler = new Handler(Looper.getMainLooper()) {
            @SuppressLint("SetTextI18n")
            @Override
            public void handleMessage(@NonNull Message msg) {
                super.handleMessage(msg);
                switch (msg.what) {
                    case 1: //开机校验更新回传
                        break;
                    case 2:  // 反馈回传
                        break;
                    case 3:  //MQTT 收到消息回传   UTF8Buffer msg=new UTF8Buffer(object.toString());
                        String message = msg.obj.toString();
                        String rx=msg.obj.toString();
                        System.out.println(rx);

                        Log.d("nicecode", "handleMessage: "+ message);
                        try {
                            JSONObject jsonObjectALL = new JSONObject(message);
                            JSONObject items = jsonObjectALL.getJSONObject("items");



                            if (items.has("temp")) {
                                temperature = items.getJSONObject("temp").getDouble("value");
                                tv_temp.setText(String.valueOf(temperature));
                                Log.d("nicecode", "temp: " + temperature);
                            }

                            // 2. 安全解析湿度
                            if (items.has("hum")) {
                                humidity = items.getJSONObject("hum").getDouble("value");
                                tv_humi.setText(String.valueOf(humidity));
                                Log.d("nicecode", "humi: " + humidity);
                            }

                            // 3. 安全解析水流 (如果云端没发，这里会自动跳过，不再引发崩溃)
                            if (items.has("Water_Flow")) {
                                Water_Flow = items.getJSONObject("Water_Flow").getDouble("value");
                                tv_Water_Flow.setText(String.valueOf(Water_Flow));
                                Log.d("nicecode", "Water_Flow: " + Water_Flow);
                            }

                            if (items.has("LightLux")) {
                                LightLux = items.getJSONObject("LightLux").getDouble("value");
                                tv_LightLux.setText(String.valueOf(LightLux));
                                Log.d("nicecode", "LightLux: " + LightLux);
                            }
                            if (items.has("ultrasound_distance")) {
                                ultrasound_distance = items.getJSONObject("ultrasound_distance").getDouble("value");
                                tv_ultrasound_distance.setText(String.valueOf(ultrasound_distance));
                                Log.d("nicecode", "ultrasound_distance: " + ultrasound_distance);
                            }
//                            tv_LightLux
                            // 4. 安全解析压力
//                            if (items.has("Pressure")) {
//                                Pressure = items.getJSONObject("Pressure").getDouble("value");
//                                tV_Pressure.setText(String.valueOf(Pressure));
//                                Log.d("nicecode", "Pressure: " + Pressure);
//                            }
                            if (items.has("Euler_angle_Roll")) {
                                tv_roll.setText(String.valueOf(items.getJSONObject("Euler_angle_Roll").getDouble("value")));
                            }
                            if (items.has("Euler_angle_Pitch")) {
                                tv_pitch.setText(String.valueOf(items.getJSONObject("Euler_angle_Pitch").getDouble("value")));
                            }
                            if (items.has("Euler_angle_Yaw")) {
                                tv_yaw.setText(String.valueOf(items.getJSONObject("Euler_angle_Yaw").getDouble("value")));
                            }
                            // 5. 安全解析烟雾浓度
                            if (items.has("smoke_density")) {
                                double smoke_density = items.getJSONObject("smoke_density").getDouble("value");
                                // 前提是你在 onCreate 里没有把 tv_smoke_density 注释掉
                                if (tv_smoke_density != null) {
                                    tv_smoke_density.setText(String.valueOf(smoke_density));
                                }
                            }
//                            JSONObject obj_LED_1 = items.getJSONObject("LED_1");
//                            JSONObject obj_LightLux = items.getJSONObject("LightLux");
//                            JSONObject obj_smoke_density = items.getJSONObject("smoke_density");
//                            JSONObject obj_ultrasound_distance = items.getJSONObject("ultrasound_distance");
//                            JSONObject obj_fan = items.getJSONObject("fan");
//                            JSONObject obj_beep = items.getJSONObject("beep");
//                            JSONObject obj_tempset = items.getJSONObject("tempset");
//                            JSONObject obj_humset = items.getJSONObject("humset");
//                            JSONObject obj_liset = items.getJSONObject("liset");

//                            smoke_density = obj_smoke_density.getDouble("value");
//                            LightLux = obj_LightLux.getInt("value");
//                            LED_1 = obj_LED_1.getInt("value");
//                            ultrasound_distance = obj_ultrasound_distance.getDouble("value");
                            //steering_angle
//                            fan = obj_fan.getInt("value");
//                            beep = obj_beep.getInt("value");
//                            tempset = obj_tempset.getInt("value");
//                            humset = obj_humset.getInt("value");
//                            liset = obj_liset.getInt("value")

//                            tv_LUX.setText(LUX + "");
//                            tv_ppm.setText(ppm + "");
//                            tv_LED_1.setText(LED_1 + "");
//                            tv_LightLux.setText(LightLux+"");
//                            tv_smoke_density.setText(smoke_density+"");
//                            tv_ultrasound_distance.setText( ultrasound_distance+"");
//                            tv_tempset.setText(tempset + "");
//                            tv_humset.setText(humset + "");
//                            tv_liset.setText(liset + "");
//                            if(LED_1==0) {
//                                tv_LED_1.setText("关");
//                                Log.d("nicecode", "LED: 关");
//                                //ledSwitch.setImageResource(R.drawable.ledoff);
//                            }
//                            else {
//                                tv_LED_1.setText("开");
//                                Log.d("nicecode", "LED: 开");
//                                //ledSwitch.setImageResource(R.drawable.ledon);
//                            }
//                            if(fan==0) {
//                                tv_fan.setText("关");
//                                Log.d("nicecode", "fan: 关");
//                                fanSwitch.setImageResource(R.drawable.fan_off);
//                            }
//                            else {
//                                tv_fan.setText("开");
//                                Log.d("nicecode", "fan: 开");
//                                fanSwitch.setImageResource(R.drawable.fan_on);
//                            }
//                            if (beep==0) {
//                                tv_beep.setText("关");
//                                Log.d("nicecode", "beep: 关");
//                                beepSwitch.setImageResource(R.drawable.beep_off);
//                            } else {
//                                tv_beep.setText("开");
//                                Log.d("nicecode", "beep: 开");
//                                beepSwitch.setImageResource(R.drawable.beep_on);
//                            }
                            Log.d("nicecode", "temp: "+ temperature);
                            Log.d("nicecode", "humi: "+ humidity);
                            Log.d("nicecode", "Water_Flow: "+ Water_Flow);
                            Log.d("nicecode", "LightLux: "+ LightLux);
                            Log.d("nicecode", "ultrasound_distance: "+ ultrasound_distance);
//                            Log.d("nicecode", "Pressure: "+ Pressure);
//                            Log.d("nicecode", "ppm: "+ ppm);
//                            Log.d("nicecode", "LUX: "+ LUX);
//                            Log.d("nicecode", "tempset: "+ tempset);
//                            Log.d("nicecode", "humset: "+ humset);
//                            Log.d("nicecode", "liset: "+ liset);
                        } catch (JSONException e) {
                            Log.e("nicecode", "JSON 解析失败", e); // 打印具体的错误栈，方便以后排查
                        }
                        break;
                    case 30:  //连接失败
                        Toast.makeText(MQTT_ALYActivity.this, "连接失败", Toast.LENGTH_SHORT).show();
                        break;
                    case 31:   //连接成功
                        Toast.makeText(MQTT_ALYActivity.this, "连接成功", Toast.LENGTH_SHORT).show();
                        try {
                            client.subscribe(sub_topic, 0);
                        } catch (MqttException e) {
                            e.printStackTrace();
                        }
                        break;
                    default:
                        break;
                }
            }
        };
    }

    private void mqtt_init() {
        try {
            String productKey = BuildConfig.PRODUCT_KEY;
            String deviceName = BuildConfig.DEVICE_NAME;
            String deviceSecret = BuildConfig.DEVICE_SECRET;
            String clientId = productKey + "." + deviceName;
            Map<String, String> params = new HashMap<>(16);
            params.put("productKey", productKey);
            params.put("deviceName", deviceName);
            params.put("clientId", clientId);
            String timestamp = String.valueOf(System.currentTimeMillis());
            params.put("timestamp", timestamp);
            String host_url = "tcp://" + productKey + ".iot-as-mqtt.cn-shanghai.aliyuncs.com:1883";
            String client_id = clientId + "|securemode=2,signmethod=hmacsha1,timestamp=" + timestamp + "|";
            String user_name = deviceName + "&" + productKey;
            String password = com.example.myapplication.AliyunIoTSignUtil.sign(params, deviceSecret, "hmacsha1");
            System.out.println(">>>" + host_url);
            System.out.println(">>>" + client_id);
            client = new MqttClient(host_url, client_id, new MemoryPersistence());
            //MQTT的连接设置
            options = new MqttConnectOptions();
            //设置是否清空session,这里如果设置为false表示服务器会保留客户端的连接记录，这里设置为true表示每次连接到服务器都以新的身份连接
            options.setCleanSession(false);
            //设置连接的用户名
            options.setUserName(user_name);
            //设置连接的密码
            options.setPassword(password.toCharArray());
            // 设置超时时间 单位为秒
            options.setConnectionTimeout(30);
            // 设置会话心跳时间 单位为秒 服务器会每隔1.5*20秒的时间向客户端发送个消息判断客户端是否在线，但这个方法并没有重连的机制
            options.setKeepAliveInterval(60);
            //设置回调
            client.setCallback(new MqttCallback() {
                @Override
                public void connectionLost(Throwable cause) {
                    //连接丢失后，一般在这里面进行重连
                    System.out.println("connectionLost----------");
                }

                @Override
                public void deliveryComplete(IMqttDeliveryToken token) {
                    //publish后会执行到这里
                    System.out.println("deliveryComplete---------" + token.isComplete());
                }

                @Override
                public void messageArrived(String topicName, MqttMessage message) {
                    //subscribe后得到的消息会执行到这里面
                    System.out.println("messageArrived----------");
                    Message msg = new Message();
                    //封装message包
                    msg.what = 3;   //收到消息标志位
                    msg.obj =message.toString();
                    //发送messge到handler
                    handler.sendMessage(msg);    // hander 回传
                }

            });
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void mqtt_connect() {
        new Thread(() -> {
            try {
                if (!(client.isConnected()))  //如果还未连接
                {
                    client.connect(options);
                    Message msg = new Message();
                    msg.what = 31;
                    // 没有用到obj字段
                    handler.sendMessage(msg);
                }
            } catch (Exception e) {
                e.printStackTrace();
                Message msg = new Message();
                msg.what = 30;
                // 没有用到obj字段
                handler.sendMessage(msg);
            }
        }).start();
    }
    private void start_reconnect() {
        ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
        scheduler.scheduleWithFixedDelay(() -> {
            if (!client.isConnected()) {
                mqtt_connect();
            }
        }, 0, 10 * 1000, TimeUnit.MILLISECONDS);
    }
    //    如果需要传输数据点这个
    private void publish_message(String message) {
        if (client == null || !client.isConnected()) {
            return;
        }
        MqttMessage mqtt_message = new MqttMessage();
        mqtt_message.setPayload(message.getBytes());
        try {
            //发送
            String pub_topic = "/sys/k1tjfOebU45/APP_dev/thing/event/property/post";
            client.publish(pub_topic, mqtt_message);
        } catch (MqttException e) {
            e.printStackTrace();
        }
    }


//    public TextView getTv_smoke_density() {
//        return tv_smoke_density;
//    }
//
//    public void setTv_smoke_density(TextView tv_smoke_density) {
//        this.tv_smoke_density = tv_smoke_density;
//    }
}