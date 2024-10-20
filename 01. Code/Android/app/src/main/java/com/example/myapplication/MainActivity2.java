package com.example.myapplication;

import static androidx.constraintlayout.helper.widget.MotionEffect.TAG;

import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.View;

import android.util.Log;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.Button;
import android.widget.ImageView;

import androidx.activity.EdgeToEdge;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.graphics.Insets;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;

import org.eclipse.paho.android.service.MqttAndroidClient;
import org.eclipse.paho.client.mqttv3.IMqttActionListener;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.json.JSONObject;

import java.io.UnsupportedEncodingException;
import java.nio.charset.StandardCharsets;

import javax.net.ssl.SSLSocketFactory;
public class MainActivity2 extends AppCompatActivity {
    private String url = "tcp://broker.hivemq.com:1883";
    private String username = "phatquserver";
    private String password = "123456789aA";
    MqttAndroidClient client;
    private Button FanNode2;
    private Button FanNode1;
    private Button LightNode1;
    private Button LightNode2;

    private TextView NhietDoNode1;
    private TextView NhietDoNode2;
    private TextView DoAmNode1;
    private TextView DoAmNode2;
    private TextView PPMNode1;
    private TextView PPMNode2;
    private TextView CoNode1;
    private TextView CoNode2;



    private ImageView Fan1;
    private ImageView Fan2;
    private ImageView Light1;
    private ImageView Light2;
    private int isFan1 = 0; // Biến để theo dõi hình ảnh hiện tại
    private boolean isFan2 = false; // Biến để theo dõi hình ảnh hiện tại
    private int isLight1 = 0; // Biến để theo dõi hình ảnh hiện tại
    private boolean isLight2 = false; // Biến để theo dõi hình ảnh hiện tại

    private final Handler handler = new Handler(Looper.getMainLooper());
    private final int SUBSCRIBE_INTERVAL = 1000; // 1 giây

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        EdgeToEdge.enable(this);
        setContentView(R.layout.activity_main2);
        Fan1 = findViewById(R.id.imageView17);
        Fan2 = findViewById(R.id.imageView20);
        Light1 = findViewById(R.id.imageView19);
        Light2 = findViewById(R.id.imageView21);

        FanNode2 = findViewById(R.id.button4);
        FanNode1 = findViewById(R.id.button6);
        LightNode1 = findViewById(R.id.button7);
        LightNode2 = findViewById(R.id.button5);

        NhietDoNode1 = findViewById(R.id.cuteTextView);
        NhietDoNode2 = findViewById(R.id.cuteTextView2);

        DoAmNode1 = findViewById(R.id.cuteTextView3);
        DoAmNode2 = findViewById(R.id.cuteTextView4);

        PPMNode1 = findViewById(R.id.cuteTextView5);
        PPMNode2 = findViewById(R.id.cuteTextView6);

        CoNode1 = findViewById(R.id.cuteTextView7);
        CoNode2 = findViewById(R.id.cuteTextView8);

        MqttConnectOptions options = new MqttConnectOptions();
        options.setUserName(username);
        options.setPassword(password.toCharArray());
        String clientId = MqttClient.generateClientId();
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                sub(); // Phương thức subscribe
                handler.postDelayed(this, SUBSCRIBE_INTERVAL); // Lặp lại sau mỗi khoảng thời gian
            }
        }, SUBSCRIBE_INTERVAL);
        FanNode1.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if (isFan1 == 0) {
//                    Fan1.setImageResource(R.drawable.fan); // Đổi sang hình ảnh thứ hai
//                    Toast.makeText(MainActivity2.this, "Changed to Image 2!", Toast.LENGTH_SHORT).show();
                    pub("{\"FAN1\": ON}");
                } else if(isFan1 == 1) {
//                    Fan1.setImageResource(R.drawable.fan__1_); // Đổi về hình ảnh ban đầu
                    pub("{\"FAN1\": OFF}");
//                    Toast.makeText(MainActivity2.this, "Changed to Image 1!", Toast.LENGTH_SHORT).show();
                }
                // Đảo ngược trạng thái
            }

        });
        FanNode2.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if (isFan2) {
                    Fan2.setImageResource(R.drawable.fan); // Đổi sang hình ảnh thứ hai
                    pub("{\"Fan2\": 1}");

//                    Toast.makeText(.this, "Changed to Image 2!", Toast.LENGTH_SHORT).show();
                } else {
                    Fan2.setImageResource(R.drawable.fan__1_); // Đổi về hình ảnh ban đầu
                    pub("{\"Fan2\": 0}");

//                    Toast.makeText(MainActivity2.this, "Changed to Image 1!", Toast.LENGTH_SHORT).show();
                }
                // Đảo ngược trạng thái
                isFan2 = !isFan2;
            }

        });
        LightNode1.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if (isLight1 == 1) {
//                    Light1.setImageResource(R.drawable.light_bulb); // Đổi sang hình ảnh thứ hai
                    pub("{\"LIGHT1\": ON}");

//                        Toast.makeText(MainActivity2.this, "Changed to Image 2!", Toast.LENGTH_SHORT).show();
                } else {
//                    Light1.setImageResource(R.drawable.house_rules); // Đổi về hình ảnh ban đầu
                    pub("{\"LIGHT1\": OFF}");
//                        Toast.makeText(MainActivity2.this, "Changed to Image 1!", Toast.LENGTH_SHORT).show();
                }
                // Đảo ngược trạng thái
            }

        });
        LightNode2.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if (isLight2) {
                    Light2.setImageResource(R.drawable.light_bulb); // Đổi sang hình ảnh thứ hai
                    pub("{\"Light2\": 1}");
//                        Toast.makeText(MainActivity2.this, "Changed to Image 2!", Toast.LENGTH_SHORT).show();
                } else {
                    Light2.setImageResource(R.drawable.house_rules); // Đổi về hình ảnh ban đầu
                    pub("{\"Light2\": 0}");

//                        Toast.makeText(MainActivity2.this, "Changed to Image 1!", Toast.LENGTH_SHORT).show();
                }
                // Đảo ngược trạng thái
                isLight2 = !isLight2;
            }

        });

        options.setServerURIs(new String[]{"ssl://" + "34f44a99a7284fa3955ec554253306f3.s1.eu.hivemq.cloud" + ":8883"});
        client = new MqttAndroidClient(this.getApplicationContext(), url, clientId);
        client.setCallback(new MqttCallback() {
            @Override
            public void connectionLost(Throwable cause) {

            }

            @Override
            public void messageArrived(String topic, MqttMessage message) throws Exception {
                // Chuyển đổi nội dung MqttMessage thành chuỗi JSON
                String messageStr = new String(message.getPayload(), StandardCharsets.UTF_8);

                // Phân tích cú pháp JSON
                JSONObject jsonObject = new JSONObject(messageStr);

                // Cập nhật giá trị cho các TextView
                NhietDoNode1.setText(jsonObject.optString("TEMP", "N/A"));
                DoAmNode1.setText(jsonObject.optString("HUMI", "N/A"));
                PPMNode1.setText(jsonObject.optString("PPM", "N/A"));
                CoNode1.setText(jsonObject.optString("CO", "N/A"));

                // Lấy giá trị boolean cho quạt và đèn
                 isFan1 = jsonObject.optInt("RL1", 0); // Nếu không có giá trị, mặc định là 0
                 isLight1 = jsonObject.optInt("RL2", 0); // Nếu không có giá trị, mặc định là 0
                if (isFan1 == 0) {
                    // Cập nhật hình ảnh cho đèn đang bật
                    Fan1.setImageResource(R.drawable.fan);
                } else {
                    // Cập nhật hình ảnh cho đèn đang tắt
                    Fan1.setImageResource(R.drawable.fan__1_);
                }
                // Cập nhật trạng thái hình ảnh cho quạt 1

                // Cập nhật trạng thái hình ảnh cho đèn 1 (nếu cần)
                if (isLight1 == 0) {
                    // Cập nhật hình ảnh cho đèn đang bật
                    Light1.setImageResource(R.drawable.light_bulb);
                } else {
                    // Cập nhật hình ảnh cho đèn đang tắt
                    Light1.setImageResource(R.drawable.house_rules);
                }
            }


            @Override
            public void deliveryComplete(IMqttDeliveryToken token) {

            }
        });
        try {
            IMqttToken token = client.connect(options);
            token.setActionCallback(new IMqttActionListener() {
                @Override
                public void onSuccess(IMqttToken asyncActionToken) {
                    // We are connected
                    Log.d(TAG, "onSuccess");
                    Toast.makeText(MainActivity2.this, "onSuccess", Toast.LENGTH_SHORT).show();
                    pub("xin chào ");
                }

                @Override
                public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                    // Something went wrong e.g. connection timeout or firewall problems
                    Log.d(TAG, "onFailure");
                    Toast.makeText(MainActivity2.this, "onFailure", Toast.LENGTH_SHORT).show();

                }
            });
        } catch (MqttException e) {
            e.printStackTrace();
        }


    }


    void pub(String content) {
        String topic = "Data/input";
        String payload = content;
        byte[] encodedPayload = new byte[0];
        try {
            encodedPayload = payload.getBytes("UTF-8");
            MqttMessage message = new MqttMessage(encodedPayload);
            client.publish(topic, message);
        } catch (UnsupportedEncodingException | MqttException e) {
            e.printStackTrace();
        }
    }

    void sub() {
        String topic = "Data";
        int qos = 1;
        try {
            IMqttToken subToken = client.subscribe(topic, qos);
            subToken.setActionCallback(new IMqttActionListener() {
                @Override
                public void onSuccess(IMqttToken asyncActionToken) {
                    // The message was published
                }

                @Override
                public void onFailure(IMqttToken asyncActionToken,
                                      Throwable exception) {
                    // The subscription could not be performed, maybe the user was not
                    // authorized to subscribe on the specified topic e.g. using wildcards

                }
            });
        } catch (MqttException e) {
            e.printStackTrace();
        }
    }
}