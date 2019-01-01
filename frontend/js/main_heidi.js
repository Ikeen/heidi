function newPage()
{
    window.location.replace('home_heidi.php');
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


//function to log the selected value of herd and animal
function selected() {

   var value_cow = document.getElementById("select_animal").value;
   var value_herd = document.getElementById("select_herd").value;
   console.log(value_cow, value_herd);

   switch(value_herd){
       case "h1":
           console.log("Herde 1 und " + value_cow + " wurde gewählt");
           break;

       case "h2":
           console.log("Herde 2 und " + value_cow + " wurde gewählt");
           break;

       case "h3":
           console.log("Herde 3 und " + value_cow + " wurde gewählt");
           break;

       case "h4":
           console.log("Herde 4 und " + value_cow + " wurde gewählt");
           break;
   }

}