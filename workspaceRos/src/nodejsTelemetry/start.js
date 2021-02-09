const app = require('./app');
require('dotenv').config();

const server = app.listen(3000, () => {
    console.log(`Node server running on port ${server.address().port}`);
});