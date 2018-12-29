<!DOCTYPE html>
<html lang="en">

<?php include("head.php"); ?>

<body class="grid">
<?php include ("menu.php"); ?>

        <div class="content">
           <!--Dropdown Herde, Dropdown Tiere als filter -> Ausgabe ist Liste mit Tieren und Status
            Button zum hinzufügen von Herden/Tieren -> Neue felder erscheinen/überblenden?/Formular -->

            <div class="dd_herd">
                <button onclick="open_dd_herd()" class="btn_herd">Herde</button>
                <div id="herd" class="dropdown-content">
                    <a href="#">Herde 1</a>
                    <a href="#">Herde 2</a>
                    <a href="#">Herde 3</a>
                </div>
            </div>

            <div class="dd_animal">
                <button onclick="open_dd_animal()" class="btn_animal">Tiere</button>
                <div id="animal" class="dropdown-content_2">
                    <a href="#">Tier 1</a>
                    <a href="#">Tier 2</a>
                    <a href="#">Tier 3</a>
                </div>
            </div>

        </div>

</body>
</html>