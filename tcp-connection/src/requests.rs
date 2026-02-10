
use embassy_time::{Duration, Timer};
use embassy_net::Stack;
use esp_println::println;
use embassy_net::tcp::TcpSocket;
use crate::types::{SERVER_IP, SERVER_PORT};

#[inline(always)]
pub async fn http_request(
    stack: &mut Stack<'_>
) {
// TCP buffers
    static mut RX_BUFFER: [u8; 1024] = [0; 1024];
    static mut TX_BUFFER: [u8; 1024] = [0; 1024];
    let mut buf = [0u8; 1024];
    let mut total_read = 0;

    let mut socket = TcpSocket::new(
        *stack,
        unsafe { &mut *core::ptr::addr_of_mut!(RX_BUFFER) },
        unsafe { &mut *core::ptr::addr_of_mut!(TX_BUFFER) },
    );
    
    loop {
        
        socket.set_timeout(Some(Duration::from_secs(10)));
        match socket.connect((SERVER_IP, SERVER_PORT)).await {
            Ok(_) => println!("Connected to server!"),
            Err(e) => {
                println!("Failed to connect: {:?}", e);
                Timer::after(Duration::from_secs(5)).await;
                continue;
            }
        }
        
        // Send HTTP GET request
        let request = "GET /health HTTP/1.1\r\nHost: 192.168.1.200:8080\r\nConnection: close\r\n\r\n";
        
        match socket.write(request.as_bytes()).await {
            Ok(_) => println!("Request sent!"),
            Err(e) => {
                println!("Failed to send request: {:?}", e);
                socket.close();
                Timer::after(Duration::from_secs(5)).await;
                continue;
            }
        }
        
        loop {
            match socket.read(&mut buf[total_read..]).await {
                Ok(0) => break,
                Ok(n) => {
                    total_read += n;
                    println!("Read {} bytes (total: {})", n, total_read);
                    if total_read >= buf.len() - 1 {
                        break;
                    }
                }
                Err(e) => {
                    println!("Read error: {:?}", e);
                    break;
                }
            }
        }
        
        // // Print response
        // if total_read > 0 {
        //     if let Ok(response) = core::str::from_utf8(&buf[..total_read]) {
        //         println!("\n=== HTTP Response ===");
        //         println!("{}", response);
        //         println!("=== End Response ===\n");
        //     } else {
        //         println!("Response (raw bytes): {:?}", &buf[..total_read]);
        //     }
        // }
        
        socket.close();
        
        // Wait before next request
        println!("Waiting 10 seconds before next request...");
        Timer::after(Duration::from_secs(1)).await;
    }
}