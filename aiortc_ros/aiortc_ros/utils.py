import socket


def have_internet(host="8.8.8.8", port=53, timeout=2):
    """Check if internet is available by pinging google's DNS server. Default timeout of 2 seconds."""
    try:
        socket.setdefaulttimeout(timeout)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        sock.close()
        return True
    except:
        return False
