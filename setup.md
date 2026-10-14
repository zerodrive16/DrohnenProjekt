# Setup 

## Tailscale 
Installation guide (Linux) and setup device and invitation links 

On PI: 
 
```
# Tailscale installation
curl -fsSL https://tailscale.com/install.sh | sh
# Tailscale start and login 
sudo tailscale up 
# Login on Web Browser 
# Tailscale status
tailscale status
# tailscale ip -4

# Login 
ssh drone@<ip_address tailscale>

```

