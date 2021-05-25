// ------------------------------------------------------
// -------------- Initialise the Database ---------------
// ------------------------------------------------------
//prefixes of implementation that we want to test
window.indexedDB = window.indexedDB || window.mozIndexedDB || 
window.webkitIndexedDB || window.msIndexedDB;

//prefixes of window.IDB objects
window.IDBTransaction = window.IDBTransaction || 
window.webkitIDBTransaction || window.msIDBTransaction;
window.IDBKeyRange = window.IDBKeyRange || window.webkitIDBKeyRange || 
window.msIDBKeyRange

if (!window.indexedDB) {
    console.error("Your browser does not support IndexedDB so you can't login.");
}

const sampleUserData = 
{
    email: "diiliev@tu-sofia.bg",
    name: "Dimitar",
    legLengthCm: 99,
    gender: "male",
    weightKg: 92,
    recommendedSeatHeightMm: 87,
    preferredSeatHeightMm: 100
};

var database;
var openDbRequest = window.indexedDB.open("ebms_db", 1);
openDbRequest.onerror = function(event) {
    console.error("Error openning database 'ebms_db'.");
}
openDbRequest.onsuccess = function(event) {
    database = openDbRequest.result;
    console.log("Successfully openned " + database);
}
openDbRequest.onupgradeneeded = function(event) {
    database = event.target.result;
    var objectStore = database.createObjectStore("ebms_users_os", {keyPath: "email"});
    objectStore.add(sampleUserData);
    console.log("Created new database and added sample user Dimitar.");
}
// ------------- Initialisation complete ----------------
// ------------------------------------------------------


// ------------------------------------------------------
// ------------- Define Database Functions --------------
// ------------------------------------------------------

function findUserByEmail(usrEmail) {
    return new Promise (function(resolve){
        var transaction = database.transaction(["ebms_users_os"], "readonly");
        var objectStore = transaction.objectStore("ebms_users_os");
        var request = objectStore.get(usrEmail);
        
        request.onerror = function(event) {
           console.error("Search request failed!");
           return resolve(null);
        };
        
        request.onsuccess = function(event) {
            if (!request.result) {
                console.error("Email: " + usrEmail +" not found in database!");
                return resolve(null);
            }
            console.log("User was found.");
            return resolve(request.result);
        };
    });
 }

// ------------------------------------------------------
