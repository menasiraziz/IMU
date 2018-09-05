package com.example.nasir.imu;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Handler;
import android.os.HandlerThread;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.TextureView;
import android.widget.TextView;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.sql.Time;
import java.util.List;
import java.util.Locale;
import java.util.Random;

public class MainActivity extends AppCompatActivity {

    private volatile boolean stop=false;
    private Handler mHandler=new Handler();
    private MyHandlerThread mHandlerThread;

    private MyHandlerThread stateHandlerThread;
    private Message message;

    private HandlerThread mSensorThread;
    private Handler mSensorHandler;

    private SensorManager sensorManager;
    List rotation;
    List pressure;
    Long timeStamp=0l;
    Long dt=0l;

    int base=1126;
    boolean change=false;
    int delta=0;
    int s1=base,s2=base,s3=base,s4=base;

    double pitch=0,roll=0;
    PID pidPitch1,pidPitch2,pidRoll1,pidRoll2;


    int k=0;
    SensorEventListener sensorEventListener=new SensorEventListener() {
        @Override

        public void onSensorChanged(SensorEvent sensorEvent) {
            if(k==1){
                k=(k+1)%2;
                return;
            }
            /*if(sensorEvent.sensor.getType()==Sensor.TYPE_ROTATION_VECTOR){
                if(message==null)
                    return;
                message.sendMessage("1100120015001300");
                mHandlerThread.postTask(message);
                return;
            }*/


            /*if(sensorEvent.sensor.getType()==Sensor.TYPE_PRESSURE){
                Log.d("PPP",""+SensorManager.getAltitude( SensorManager.PRESSURE_STANDARD_ATMOSPHERE
                        ,sensorEvent.values[0])+"\t"+dt);
            }*/
            if(sensorEvent.sensor.getType()==Sensor.TYPE_ROTATION_VECTOR){
                dt=(sensorEvent.timestamp-timeStamp)/1000000;
                float[] rotationMatrix = new float[9];
                SensorManager.getRotationMatrixFromVector(rotationMatrix, sensorEvent.values);
                float[] values=new float[3];
                SensorManager.getOrientation(rotationMatrix,values);

                int[] v=new int[3];
                v[0]=(int) (((180/Math.PI)*values[0])+180);
                v[1]=(int) ((180/Math.PI)*values[1]);
                v[2]=(int) ((180/Math.PI)*values[2]);

                Log.d("IMU",""+v[0]+"\t"+v[1]+"\t"+v[2]);
            //Log.d("IMU",""+(180/Math.PI)*values[0]+"\t"+(180/Math.PI)*values[1]+"\t"+(180/Math.PI)*values[2]);
            /*if((v[1]+5)<0){
                s1=s1+10;
                //Log.d("IMU","s1"+v[1]);
            }
             if((v[1]-5)>0){
                s1--;
            }
            if((v[2]-5)>0){
                s4=s4+1;
                s3=s3-1;
                //Log.d("IMU","s4"+v[2]);
            }
            if((v[2]+5)<0){
                s4=s4-1;
                s3=s3+1;
            }*/
                s1=pidPitch1.output(pitch,v[1], s1);
                s2=pidPitch2.output(-pitch,-v[1],s2);
                s3=pidRoll2.output(roll,v[2],s3);
                s4=pidRoll1.output(-roll,-v[2],s4);
                if(change){
                    s1=s1+delta;
                    s2=s2+delta;
                    s3=s3+delta;
                    s4=s4+delta;
                    change=false;
                }

                s1=clamp(s1);
                s2=clamp(s2);
                s3=clamp(s3);
                s4=clamp(s4);

                Log.d("IMU",""+s1+"\t"+s2+"\t"+s3+"\t"+s4);

                message.sendMessage(Integer.toString(1126)+Integer.toString(1126)+
                        Integer.toString(s3)+Integer.toString(s4));
                mHandlerThread.postTask(message);
                String b= message.receivedMessage();
                if(b==null){
                    return;
                }
                try{
                    if(b!="11"){
                        int bb=Integer.parseInt(b);
                        delta=bb;
                        change=true;

                    }

                    //Log.d("IMU",Integer.toString(bb));

                }catch (Exception e){
                    //Log.d("IMU","ex");
                    //e.printStackTrace();
                }
                    timeStamp=sensorEvent.timestamp;
            }
            //Log.d("IMU",""+(180/Math.PI)*Math.asin(2*(sensorEvent.values[0]*sensorEvent.values[2]-
             //       sensorEvent.values[1]*sensorEvent.values[3])));
            //Log.d("IMU",""+sensorEvent.values[0]+"\t"
             //       +sensorEvent.values[1]+"\t"+sensorEvent.values[2]+"\t"+sensorEvent.values[3]+"\t"+dt);

        }

        int upClamp=1500;
        int downClamp=1126;
        private int clamp(int s){
            if(s>upClamp)
                return upClamp;
            if(s<downClamp)
                return downClamp;
            return s;
        }




        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };
 
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        textView=findViewById(R.id.textView);

        mHandlerThread=new MyHandlerThread("MyHander");
        mHandlerThread.start();
        mHandlerThread.prepareHandler();
        stateHandlerThread=new MyHandlerThread("State");
        stateHandlerThread.start();
        stateHandlerThread.prepareHandler();
        message=new Message();


        mSensorThread=new HandlerThread("sensor thread");
        mSensorThread.start();
        mSensorHandler=new Handler(mSensorThread.getLooper());
        sensorManager= (SensorManager) getSystemService(SENSOR_SERVICE);
        rotation=sensorManager.getSensorList(Sensor.TYPE_ROTATION_VECTOR);
        pressure=sensorManager.getSensorList(Sensor.TYPE_PRESSURE);
        //Log.d("PPP",""+pressure.size());
        if(pressure.size()>0){
            sensorManager.registerListener(sensorEventListener, (Sensor) pressure.get(0),
                    SensorManager.SENSOR_DELAY_FASTEST,mSensorHandler);
        }
        if(rotation.size()>0){
            sensorManager.registerListener(sensorEventListener, (Sensor) rotation.get(0),
                    SensorManager.SENSOR_DELAY_FASTEST,mSensorHandler);
        }
        pidPitch1=new PID(0.1,-0.05,0.1);
        pidPitch2=new PID(0.1,-0.05,0.1);
        pidRoll1=new PID(0.3,-0.05,0.1);
        pidRoll2=new PID(0.3,-0.05,0.1);
        /*final Runnable state=new Runnable() {
            @Override
            public void run() {
                int i=0;
                while (!stop){

                    message.sendMessage(Integer.toString(i));
                    //Log.d("IMU",Integer.toString(i));
                    mHandlerThread.postTask(message);

                    //Log.d("IMU","running");
                    try {
                        Thread.sleep(1);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    i=(i+1)%10;
                }
            }
        };
        stateHandlerThread.postTask(state);*/

    }
    private TextView textView;

    @Override
    protected void onStop() {
        if(rotation.size()>0){
            sensorManager.unregisterListener(sensorEventListener);
        }
        if(pressure.size()>0){
            sensorManager.unregisterListener(sensorEventListener);
        }
        super.onStop();
    }

    @Override
    protected void onDestroy() {
        stop=true;
        stopHandlerThread(mHandlerThread);
        stopHandlerThread(stateHandlerThread);
        if(mHandler!=null)
            mHandler=null;
        message.closeSocket();
        mSensorThread.quitSafely();
        if(mSensorHandler!=null)
            mSensorHandler=null;
        super.onDestroy();
    }
    private void stopHandlerThread(MyHandlerThread thread){
        thread.quit();
        thread.interrupt();
        thread=null;


    }

    public class MyHandlerThread extends HandlerThread{
        private Handler handler;

        public MyHandlerThread(String name) {
            super(name);
        }
        public void postTask(Runnable task){
            handler.post(task);
        }
        public void prepareHandler(){
            handler=new Handler(getLooper());
        }

    }

    int REM_REC_PORT=5555;
    int LOC_SEND_PORT=4444;
    public class Message implements Runnable{

        private String sendMSG;
        private String recMSG;
        boolean received=false;

        DatagramPacket datagramPacket;
        DatagramSocket datagramSocket=null;
        InetAddress local= null;

        public Message(){
            try {
                datagramSocket=new DatagramSocket(LOC_SEND_PORT);
                datagramSocket.setSoTimeout(5);
            } catch (SocketException e) {
                //e.printStackTrace();
            }
            try {
                local = InetAddress.getByName("192.168.43.154");
            } catch (UnknownHostException e) {
                //e.printStackTrace();
            }
        }
        @Override
        public void run() {
            //Log.d("IMU",sendMSG);
            long before=System.nanoTime();
            //Log.d("IMU","Sending");

            byte[] msgBytes=sendMSG.getBytes();
            DatagramPacket datagramPacket=new DatagramPacket(msgBytes,msgBytes.length,local,REM_REC_PORT);
            try {
                if(datagramSocket!=null)
                    datagramSocket.send(datagramPacket);
            } catch (IOException e) {
                //e.printStackTrace();
            }
            byte[] bytes=new byte[100];
                datagramPacket=new DatagramPacket(bytes,bytes.length);
            try {
                if(datagramSocket!=null){
                    datagramSocket.receive(datagramPacket);
                    received=true;
                }
            } catch (IOException e) {
                //e.printStackTrace();
                //Log.d("IMU","not received"+sendMSG);
                received=false;
            }
            if(received)
                recMSG=new String(datagramPacket.getData(),datagramPacket.getOffset(),datagramPacket.getLength());
            else
                recMSG="11";

            long after=System.nanoTime();
            //if(recMSG!="11")
              //  Log.d("IMU",recMSG);
            /*double del=(double)(after-before)/1000000.0;
            if(del>4)
                Log.d("IMU",""+del);*/


        }
        public void sendMessage(String msg){
            this.sendMSG=msg;
        }
        public String receivedMessage(){
            return recMSG;
        }
        public void closeSocket(){
            datagramSocket.close();
        }
    }

    public class PID{
        private double  err1=0,err2=0,err3=0;
        private double  a,b,c;
        private int output;
        public PID(double a,double b,double c){
            this.a=a;
            this.b=b;
            this.c=c;
        }
        public int output(double ref,double curr,int input){
            err1=(ref-curr)/2;
            if(input>1400)
                output=(input+(int)(a*0.3*err1+b*err2+c*0.3*err3));
            else
                output=(input+(int)(a*err1+b*err2+c*err3));
            //Log.d("IMU",output+"\t"+err1+"\t"+curr+"\t"+ref);
            err3=err2;
            err2=err1;
            return output;
        }
    }

}
