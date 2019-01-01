<!DOCTYPE html>
<html lang="en">

<?php include("head.php"); ?>

<body class="grid">
<?php include ("menu.php"); ?>

        <div class="content">
           <!--Dropdown Herde, Dropdown Tiere als filter -> Ausgabe ist Liste mit Tieren und Status
            Button zum hinzufügen von Herden/Tieren -> Neue felder erscheinen/überblenden?/Formular -->
            <div class="dd_herd">
                <select id="select_herd" onchange="selected()">
                    <option value="h1">Herde 1</option>
                    <option value="h2">Herde 2</option>
                    <option value="h3">Herde 3</option>
                    <option value="h4">Herde 4</option>
                </select>
            </div>

            <div class="dd_animal">
                <select id="select_animal" onchange="selected()">
                    <option value="a1">Kuh 1</option>
                    <option value="a2">Kuh 2</option>
                    <option value="a3">Kuh 3</option>
                    <option value="a4">Kuh 4</option>
                </select>
            </div>

        </div>

</body>
</html>