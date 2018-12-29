function newPage()
{
    window.location.replace('home.php');
}

function greeting(){

    //personalisierte Begrüßung

}

/* Toggle between adding and removing the "responsive" class to topnav when the user clicks on the icon */
function myFunction() {
    var x = document.getElementById("respmenu");
    if (x.className === "menu") {
        x.className += " responsive";

        document.getElementsByClassName("grid").style.display = "none";

    } else {
        x.className = "menu";
    }
}

function open_dd_herd() {
    document.getElementById("herd"). classList.toggle("show");
}

window.onclick = function(event) {
    if (!event.target.matches('.btn_herd')) {

        var dropdowns = document.getElementsByClassName("dropdown-content");
        var i;
        for (i = 0; i < dropdowns.length; i++) {
            var openDropdown = dropdowns[i];
            if (openDropdown.classList.contains('show')) {
                openDropdown.classList.remove('show');
            }
        }
    }
}


function open_dd_animal() {
    document.getElementById("animal"). classList.toggle("show");
}

window.onclick = function(event) {
    if (!event.target.matches('.btn_animal')) {

        var dropdowns = document.getElementsByClassName("dropdown-content_2");
        var i;
        for (i = 0; i < dropdowns.length; i++) {
            var openDropdown = dropdowns[i];
            if (openDropdown.classList.contains('show')) {
                openDropdown.classList.remove('show');
            }
        }
    }
}