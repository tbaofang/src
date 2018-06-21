1、TCP网络编程主要流程


图1.1

注意：图1.1中可以看到close指向read并且标有结束连接的指示，可能有些人会有疑问，这个标注的意思是服务器在处理客户端的时候是循环读取的，如果客户端没有发送数据服务器处理客户端的线程是阻塞在read这里的，当客户端调用close后，服务器read就会立刻返回-1，这时服务器处理线程才会继续向下执行。如果客户端没有执行close而是直接异常退出，那么服务器端的read会立即返回0，所以在read后面一定要判断返回值，根据不同返回值进行不同的处理，这些是在试实验中总结的经验。

2、结构及相关函数

(1) struct sockaddr(套接字的普通C定义通用的地址结构)

structsockaddr {

u_charsa_len;//长度

u_short  sa_family;//协议

char  sa_data[14];//数据

};

(2) struct sockaddr_in(IP专用的地址结构)

structsockaddr_in {

u_char    sin_len;//长度

u_short   sin_family；//协议

u_short    sin_port；//端口

structin_addr   sin_addr;//ip地址

char   sin_zero[8];//数据

};

(3) struct in_addr

structin_addr {

           u_longs_addr;

};

用来表示一个32位的IPv4地址，其字节顺序为网络顺序。

(4) int Socket( int domain, int type,int protocol)

功能：创建一个新的套接字，返回套接字描述符

参数说明：

domain：域类型，指明使用的协议栈，如TCP/IP使用的是 PF_INET    ，其他还有AF_INET6、AF_UNIX

type:指明需要的服务类型, 如

SOCK_DGRAM:数据报服务，UDP协议

SOCK_STREAM:流服务，TCP协议

protocol:一般都取0(由系统根据服务类型选择默认的协议)

(5) int bind(int sockfd,struct sockaddr* my_addr,int addrlen) 功能：为套接字指明一个本地端点地址

TCP/IP协议使用sockaddr_in结构，包含IP地址和端口号，服务器使用它来指明熟知的端口号，然后等待连接

参数说明：

sockfd:套接字描述符，指明创建连接的套接字

my_addr:本地地址，IP地址和端口号

addrlen:地址长度

套接口中port=0表示由内核指定端口号，设定sin_addr为INADDR_ANY，由内核指定IP地址。

(6) int listen(int sockfd,intinput_queue_size)

功能：

面向连接的套接字使用它将一个套接字置为被动模式，并准备接收传入连接。用于服务器，指明某个套接字连接是被动的

参数说明：

Sockfd:套接字描述符，指明创建连接的套接字

input_queue_size:该套接字使用的队列长度,指定在请求队列中允许的最大请求数

(7) int accept(int sockfd, structsockaddr *addr, int *addrlen)

功能：获取传入连接请求，返回新的连接的套接字描述符。

为每个新的连接请求创建了一个新的套接字，服务器只对新的连接使用该套接字，原来的监听套接字接收其他的连接请求。新的连接上传输数据使用新的套接字，使用完毕，服务器将关闭这个套接字。

参数说明：

Sockfd:套接字描述符，指明正在监听的套接字

addr:提出连接请求的主机地址

addrlen:地址长度

(8) int connect(int sockfd,structsockaddr *server_addr,int sockaddr_len)

功能： 同远程服务器建立主动连接，成功时返回0，若连接失败返回－1。

参数说明：

Sockfd:套接字描述符，指明创建连接的套接字

Server_addr:指明远程端点：IP地址和端口号

sockaddr_len :地址长度

(9)int send(int sockfd, const void * data, int data_len, unsigned int flags)

功能：

在TCP连接上发送数据,返回成功传送数据的长度，出错时返回－1。send会将外发数据复制到OS内核中，也可以使用send发送面向连接的UDP报文。

参数说明：

sockfd:套接字描述符

data:指向要发送数据的指针

data_len:数据长度

flags:通常为0

如果send()函数的返回值小于len的话，则你需要再次发送剩下的数据。802.3，MTU为1492B，如果包小于1K，那么send()一般都会一次发送光的。

(10) int recv(int sockfd, void *buf, intbuf_len,unsigned int flags)

功能：

从TCP接收数据,返回实际接收的数据长度，出错时返回－1。

服务器使用其接收客户请求，客户使用它接受服务器的应答。如果没有数据，将阻塞。如果TCP收到的数据大于(/小于)缓存的大小，只抽出能够填满缓存的足够数据(/抽出所有数据并返回它实际接收的字节数)。也可以使用recv接收面向连接的UDP的报文，若缓存不能装下整个报文，填满缓存后剩下的数据将被丢弃。

参数说明：

Sockfd:套接字描述符

Buf:指向内存块的指针

Buf_len:内存块大小，以字节为单位

flags:一般为0(MSG_WAITALL接收到指定长度数据时才返回)，设置为 MSG_DONTWAIT为非阻塞

 (11)close(int sockfd)

功能：

撤销套接字.如果只有一个进程使用，立即终止连接并撤销该套接字，如果多个进程共享该套接字，将引用数减一，如果引用数降到零，则关闭连接并撤销套接字。

参数说明：

sockfd:套接字描述符

 3、TCPSocket客户服务器通信实例

下面通过一个简单的例子来形象地演示图1.1中所示的过程，服务器等待客户端连接，连接后等待客户端发送数据，服务器收到客户端发来的数据之后再发送回去，一个“回发”的功能。
