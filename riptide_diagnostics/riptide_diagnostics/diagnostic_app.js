document.addEventListener('DOMContentLoaded', function() {
    const displayElement = document.getElementById('dataDisplay');

    const url = "ws://localhost:8080";  // Connect to the WebSocket server running at this URL
    const socket = new WebSocket(url);

    socket.onopen = function(event) {
        console.log('Connected to:', url);
    };

    socket.onmessage = function(event) {
        console.log('Message from server:', event.data);
        displayElement.textContent = JSON.stringify(JSON.parse(event.data), null, 4);  // Display nicely formatted JSON
    };

    socket.onerror = function(error) {
        console.error('WebSocket error:', error);
    };

    socket.onclose = function(event) {
        console.log('WebSocket connection closed:', event);
    };
});
