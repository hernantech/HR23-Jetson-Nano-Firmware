Instructions for connecting to the nano’s jupyterlab servers

Ssh into the nano

Set up a jupyterlab instance with:

For example, here we’ll set it up on port 1234, and route it to our local computer’s port 1234 via ssh. If the port is in use then you can change the local or remote port.
```
jupyter lab --port=1234>--no-browser
```
Do not close this window as you still need the token (probably, at least).

Open a new terminal and route the port to your local machine:
```
ssh -L 1234:localhost:1234 nanoUsername@nanoIPaddress
```
Jupyterlab may ask for a token, either request it via ssh (using the Jupyterlab commands) or copy it from the other terminal window
i.e. $jupyter server list
