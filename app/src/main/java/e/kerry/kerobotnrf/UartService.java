package e.kerry.kerobotnrf;

import android.app.Service;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.BluetoothLeScanner;
import android.content.Context;
import android.content.Intent;
import android.os.Binder;
import android.os.IBinder;
import android.util.Log;

import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import java.util.List;
import java.util.UUID;

/**
 * Service for managing connection and data communication with a GATT server hosted on a
 * given Bluetooth LE device.
 */
public class UartService extends Service {
    private final static String TAG = UartService.class.getSimpleName();

    private BluetoothManager mBluetoothManager;
    private BluetoothAdapter mBluetoothAdapter;
    private BluetoothLeScanner mBluetoothLeScanner;
    private String mBluetoothDeviceAddress;
    private BluetoothGatt mBluetoothGatt;
    private BluetoothGatt gatt;
    private BluetoothGattService RxService;
    private BluetoothGattCharacteristic TxChar;
    private BluetoothGattCharacteristic AckChar;
    private BluetoothGattDescriptor TxDescriptor;
    private BluetoothGattDescriptor AckDescriptor;
    private int mConnectionState = STATE_DISCONNECTED;
    byte [] DescriptorUuid = {(byte) 2902};
    private static final int STATE_DISCONNECTED = 0;
    private static final int STATE_CONNECTING = 1;
    private static final int STATE_CONNECTED = 2;

    public final static String ACTION_GATT_CONNECTED =
            "com.nordicsemi.nrfUART.ACTION_GATT_CONNECTED";
    public final static String ACTION_GATT_DISCONNECTED =
            "com.nordicsemi.nrfUART.ACTION_GATT_DISCONNECTED";
    public final static String ACTION_GATT_SERVICES_DISCOVERED =
            "com.nordicsemi.nrfUART.ACTION_GATT_SERVICES_DISCOVERED";
    public final static String ACTION_DATA_AVAILABLE =
            "com.nordicsemi.nrfUART.ACTION_DATA_AVAILABLE";
    public final static String EXTRA_DATA =
            "com.nordicsemi.nrfUART.EXTRA_DATA";
    public final static String DEVICE_DOES_NOT_SUPPORT_UART =
            "com.nordicsemi.nrfUART.DEVICE_DOES_NOT_SUPPORT_UART";

    //public static final UUID TX_POWER_UUID = UUID.fromString("00001804-0000-1000-8000-00805f9b34fb");
    //public static final UUID TX_POWER_LEVEL_UUID = UUID.fromString("00002a07-0000-1000-8000-00805f9b34fb");
    //public static final UUID FIRMWARE_REVISON_UUID = UUID.fromString("00002a26-0000-1000-8000-00805f9b34fb");
    //public static final UUID DIS_UUID = UUID.fromString("0000180a-0000-1000-8000-00805f9b34fb");
    public static final UUID ACK_CCCDESCRITPTOR = UUID.fromString("000002902-0000-1000-8000-00805f9b34fb");
    public static final UUID TX_CCCDESCRITPTOR = UUID.fromString("000002902-0000-1000-8000-00805f9b34fb");
    //public static final UUID RX_SERVICE_UUID = UUID.fromString("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
    //public static final UUID RX_CHAR_UUID = UUID.fromString("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
    //public static final UUID TX_CHAR_UUID = UUID.fromString("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
    public static final UUID RX_SERVICE_UUID = UUID.fromString("00002760-08c2-11e1-9073-0e8ac72e1011");
    public static final UUID RX_CHAR_UUID = UUID.fromString("00002760-08c2-11e1-9073-0e8ac72e0011");
    public static final UUID TX_CHAR_UUID = UUID.fromString("00002760-08c2-11e1-9073-0e8ac72e0012");
    public static final UUID ACK_CHAR_UUID = UUID.fromString("00002760-08c2-11e1-9073-0e8ac72e0013");
    public static final byte [] ACK_Value = {(byte) 0x05, (byte) 0x00, (byte) 0x00, (byte) 0x20,
            (byte) 0x00, (byte) 0x8d, (byte) 0xef, (byte) 0x02, (byte) 0xd2};
    public int i_CharDesc = 0;


    // Implements callback methods for GATT events that the app cares about.  For example,
    // connection change and services discovered.
    private final BluetoothGattCallback mGattCallback = new BluetoothGattCallback() {
        @Override
        public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
            String intentAction;

            if (newState == BluetoothProfile.STATE_CONNECTED) {
                intentAction = ACTION_GATT_CONNECTED;
                mConnectionState = STATE_CONNECTED;

                broadcastUpdate(intentAction);
                //gatt.requestMtu(247); //Kills Moto G6 Android Pie logon
                Log.i(TAG, "KHE Connected to GATT server.");
                // Attempts to discover services after successful connection.
                Log.i(TAG, "KHE Attempting to start service discovery:" +
                        mBluetoothGatt.discoverServices());

            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                intentAction = ACTION_GATT_DISCONNECTED;
                mConnectionState = STATE_DISCONNECTED;
                Log.i(TAG, "Disconnected from GATT server.");
                broadcastUpdate(intentAction);
            }
        }
        //@Override
        //public void
        @Override
        public void onServicesDiscovered(BluetoothGatt gatt, int status) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                Log.w(TAG, "mBluetoothGatt = " + mBluetoothGatt );

                broadcastUpdate(ACTION_GATT_SERVICES_DISCOVERED);
            } else {
                Log.w(TAG, "onServicesDiscovered received: " + status);
            }
        }

        @Override
        public void onCharacteristicRead(BluetoothGatt gatt,
                                         BluetoothGattCharacteristic characteristic,
                                         int status) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                broadcastUpdate(ACTION_DATA_AVAILABLE, characteristic);
            }
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt gatt,
                                            BluetoothGattCharacteristic characteristic) {
            //if (characteristic.getUuid() == ((UUID)ACK_CHAR_UUID)){
            //writeRXCharacteristic(ACK_Value);
            //}SET BREAKPOINT ON NEXT LINE JUMP
            broadcastUpdate(ACTION_DATA_AVAILABLE, characteristic);

            //Callback triggered as a result of a remote characteristic notification.
        }
    };

    private void broadcastUpdate(final String action) {
        final Intent intent = new Intent(action);
        LocalBroadcastManager.getInstance(this).sendBroadcast(intent);
    }

    private void broadcastUpdate(final String action,
                                 final BluetoothGattCharacteristic characteristic) {
        final Intent intent = new Intent(action);

        // This is handling for the notification on TX Character of NUS service
        if (TX_CHAR_UUID.equals(characteristic.getUuid())) {

            // Log.d(TAG, String.format("Received TX: %d",characteristic.getValue() ));
            intent.putExtra(EXTRA_DATA, characteristic.getValue());
        } else {

        }
        LocalBroadcastManager.getInstance(this).sendBroadcast(intent);
    }

    public class LocalBinder extends Binder {
        UartService getService() {
            return UartService.this;
        }
    }

    @Override
    public IBinder onBind(Intent intent) {
        return mBinder;
    }

    @Override
    public boolean onUnbind(Intent intent) {
        // After using a given device, you should make sure that BluetoothGatt.close() is called
        // such that resources are cleaned up properly.  In this particular example, close() is
        // invoked when the UI is disconnected from the Service.
        close();
        return super.onUnbind(intent);
    }

    private final IBinder mBinder = new LocalBinder();

    /**
     * Initializes a reference to the local Bluetooth adapter.
     *
     * @return Return true if the initialization is successful.
     */
    public boolean initialize() {
        // For API level 18 and above, get a reference to BluetoothAdapter through
        // BluetoothManager.
        if (mBluetoothManager == null) {
            mBluetoothManager = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
            if (mBluetoothManager == null) {
                Log.e(TAG, "Unable to initialize BluetoothManager.");
                return false;
            }
        }

        mBluetoothAdapter = mBluetoothManager.getAdapter();
        if (mBluetoothAdapter == null) {
            Log.e(TAG, "Unable to obtain a BluetoothAdapter.");
            return false;
        }

        return true;
    }

    /**
     * Connects to the GATT server hosted on the Bluetooth LE device.
     *
     * @param address The device address of the destination device.
     *
     * @return Return true if the connection is initiated successfully. The connection result
     *         is reported asynchronously through the
     *         {@code BluetoothGattCallback#onConnectionStateChange(android.bluetooth.BluetoothGatt, int, int)}
     *         callback.
     */
    public boolean connect(final String address) {
        if (mBluetoothAdapter == null || address == null) {
            Log.w(TAG, "BluetoothAdapter not initialized or unspecified address.");
            return false;
        }

        // Previously connected device.  Try to reconnect.
        if (mBluetoothDeviceAddress != null && address.equals(mBluetoothDeviceAddress)
                && mBluetoothGatt != null) {
            Log.d(TAG, "Trying to use an existing mBluetoothGatt for connection.");
            if (mBluetoothGatt.connect()) {
                mConnectionState = STATE_CONNECTING;
                return true;
            } else {
                return false;
            }
        }

        final BluetoothDevice device = mBluetoothAdapter.getRemoteDevice(address);
        if (device == null) {
            Log.w(TAG, "Device not found.  Unable to connect.");
            return false;
        }
        // We want to directly connect to the device, so we are setting the autoConnect
        // parameter to false.
        //KHE - changed next line from false to true
        mBluetoothGatt = device.connectGatt(this, false, mGattCallback);
        //KHE - all messages from login are done as of this point
        Log.d(TAG, "Trying to create a new connection.");
        mBluetoothDeviceAddress = address;
        mConnectionState = STATE_CONNECTING;
        return true;
    }

    /**
     * Disconnects an existing connection or cancel a pending connection. The disconnection result
     * is reported asynchronously through the
     * {@code BluetoothGattCallback#onConnectionStateChange(android.bluetooth.BluetoothGatt, int, int)}
     * callback.
     */
    public void disconnect() {
        if (mBluetoothAdapter == null || mBluetoothGatt == null) {
            Log.w(TAG, "BluetoothAdapter not initialized");
            return;
        }
        mBluetoothGatt.disconnect();
        // mBluetoothGatt.close();
    }

    /**
     * After using a given BLE device, the app must call this method to ensure resources are
     * released properly.
     */
    public void close() {
        if (mBluetoothGatt == null) {
            return;
        }
        Log.w(TAG, "mBluetoothGatt closed");
        mBluetoothDeviceAddress = null;
        mBluetoothGatt.close();
        mBluetoothGatt = null;
    }

    /**
     * Request a read on a given {@code BluetoothGattCharacteristic}. The read result is reported
     * asynchronously through the {@code BluetoothGattCallback#onCharacteristicRead(android.bluetooth.BluetoothGatt, android.bluetooth.BluetoothGattCharacteristic, int)}
     * callback.
     *
     * @param characteristic The characteristic to read from.
     */
    public void readCharacteristic(BluetoothGattCharacteristic characteristic) {
        if (mBluetoothAdapter == null || mBluetoothGatt == null) {
            Log.w(TAG, "BluetoothAdapter not initialized");
            return;
        }
        mBluetoothGatt.readCharacteristic(characteristic);
    }

    /**
     * Enables or disables notification on a give characteristic.
     *

     */

    /**
     * Enable Notification on TX characteristic
     *
     * @return
     */
    public boolean enableTXNotification()
    {
        boolean b_Success = false;
        RxService = mBluetoothGatt.getService(RX_SERVICE_UUID);
        TxChar = RxService.getCharacteristic(TX_CHAR_UUID);
        TxChar.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);
        mBluetoothGatt.setCharacteristicNotification(TxChar,true);
        TxDescriptor = TxChar.getDescriptor(TX_CCCDESCRITPTOR);
        TxDescriptor.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE); //= 0x01 0x00
        b_Success = mBluetoothGatt.writeDescriptor(TxDescriptor);
        return b_Success;
    }
    public boolean enableAckNotification()
    {
        //JUMP
        boolean b_Success = false;
        RxService = mBluetoothGatt.getService(RX_SERVICE_UUID);
        AckChar = RxService.getCharacteristic(ACK_CHAR_UUID);
        AckChar.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);
        if(mBluetoothGatt.setCharacteristicNotification(AckChar,true))
            AckDescriptor = AckChar.getDescriptor(ACK_CCCDESCRITPTOR);
        AckDescriptor.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
        //= 0x01 0x00
        b_Success =  mBluetoothGatt.writeDescriptor(AckDescriptor);
        return b_Success;
    }

    public void writeTx(){
        mBluetoothGatt.writeDescriptor(TxDescriptor);
    }

    public void writeAck(){
        mBluetoothGatt.writeDescriptor((AckDescriptor));
    }

    public void writeRXCharacteristic(byte[] value)
    {

        // arrives here after entering number.  Enter number as utf-8 converted then here
        BluetoothGattService RxService = mBluetoothGatt.getService(RX_SERVICE_UUID);
        showMessage("mBluetoothGatt null"+ mBluetoothGatt);
        if (RxService == null) {
            showMessage("Rx service not found!");
            broadcastUpdate(DEVICE_DOES_NOT_SUPPORT_UART);
            return;
        }
        BluetoothGattCharacteristic RxChar = RxService.getCharacteristic(RX_CHAR_UUID);
        if (RxChar == null) {
            showMessage("Rx charateristic not found!");
            broadcastUpdate(DEVICE_DOES_NOT_SUPPORT_UART);
            return;
        }
        //Trying to set offset = 0 value[2] = 0;
        RxChar.setValue(value);
        boolean status = mBluetoothGatt.writeCharacteristic(RxChar);


        Log.d(TAG, "write TXchar - status=" + status);
    }

    private void showMessage(String msg) {
        Log.e(TAG, msg);
    }
    /**
     * Retrieves a list of supported GATT services on the connected device. This should be
     * invoked only after {@code BluetoothGatt#discoverServices()} completes successfully.
     *
     * @return A {@code List} of supported services.
     */
    public List<BluetoothGattService> getSupportedGattServices() {
        if (mBluetoothGatt == null) return null;

        return mBluetoothGatt.getServices();
    }
}
