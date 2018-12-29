<!DOCTYPE html>
<html lang="en">

<?php include("head.php"); ?>

<body>
    <div class="box">
        <div id="logo">
            <h1>Heidi</h1>
            <img id="cow" src="../img/Logo.png">
        </div>
        <div id="login">
            <input id="name" type="text" name="name" placeholder="Name">
            <br>
            <input id="pw" type="password" name="password" placeholder="Passwort">
            <br>
            <!-- Überarbeiten und noch mehr Sicherheit-->
            <input id="button" type="submit" value="Muhh" onclick=newPage()>
        </div>
    </div>
</body>
<footer>
    <img id="gras" src="../img/gras.png">
</footer>
</html>