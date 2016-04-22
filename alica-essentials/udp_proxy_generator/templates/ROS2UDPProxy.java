package util;

import android.app.Activity;

import com.github.ros_java.marauders_map.R;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageDeserializer;
import org.ros.message.MessageListener;
import org.ros.message.MessageSerializer;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.net.MulticastSocket;
import java.util.Arrays;
import java.nio.ByteBuffer;
import java.util.Properties;
import java.nio.ByteOrder;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.net.Inet4Address;


<?messageIncludes?>

/**
 * Created by marci on 22.10.15.
 */
public class ROS2UDPProxy implements NodeMain {

    private Activity activity;

    public java.lang.String ownRosName;

    private ConnectedNode node;
    
    private MulticastSocket udpSocket;
    
    private InetAddress group;
    
    private int port;
    
    private InetAddress localhost;

    public InetAddress getGroup() {
        return group;
    }

    public int getPort() {
        return port;
    }
    
    <?rosMessageHandler?>
    
    <?rosPublisherDecl?>

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("MaraudersMap/Proxy");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        Properties udpConfig = new Properties();
        try {
        <?configfile?>
		java.lang.String multiCastAdress = udpConfig.getProperty("MulticastAddress");
        port = Integer.parseInt(udpConfig.getProperty("Port"));
        udpSocket = new MulticastSocket(port);
        group = InetAddress.getByName(multiCastAdress);
        udpSocket.joinGroup(group);
        udpSocket.setLoopbackMode(false);
        node = connectedNode;
        
        <?subscriptions?>
        
        <?advertisement?>

        listenForPacket(udpSocket);
        ownRosName = connectedNode.getName().toString();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    void listenForPacket(final MulticastSocket socket) {
        byte[] buffer = new byte[64000];
        
        try {
			Enumeration<NetworkInterface> list = NetworkInterface.getNetworkInterfaces();

			while(list.hasMoreElements() && localhost == null) {
				NetworkInterface networkInterface = list.nextElement();
				if(networkInterface.getName().contains("wlan")) {
					Enumeration<InetAddress> list2 = networkInterface.getInetAddresses();

					do {
						InetAddress address = list2.nextElement();
						if(address instanceof Inet4Address) {
							localhost = address;
						}
					} while (list2.hasMoreElements() && localhost == null);
				}
			}
		} catch (SocketException e) {
			e.printStackTrace();
		}
        
        final DatagramPacket packet = new DatagramPacket(buffer,buffer.length);
        node.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                try {
                    socket.receive(packet);
                    handleUdpPacket(packet);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        });
    }

    @Override
    public void onShutdown(Node node) {

    }

    @Override
    public void onShutdownComplete(Node node) {

    }

    @Override
    public void onError(Node node, Throwable throwable) {
    }

    public Activity getActivity() {
        return activity;
    }

    public void setActivity(Activity activity) {
        this.activity = activity;
    }

    void handleUdpPacket(DatagramPacket packet) {
    	if(packet.getAddress().equals(localhost)) {
			return;
		}
		long id = ChannelBuffers.copiedBuffer(ByteOrder.LITTLE_ENDIAN,Arrays.copyOfRange(packet.getData(), 0, 4)).getUnsignedInt(0);
        	<?udpReception?>
            else {
                System.err.println("Cannot find Matching topic:" + id);
            }
    }
}
