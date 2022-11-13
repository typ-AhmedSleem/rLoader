module com.typ.rloader.wcu {
    requires javafx.controls;
    requires javafx.fxml;
    requires javafx.web;
    requires kotlin.stdlib;

    requires org.controlsfx.controls;
    requires eu.hansolo.tilesfx;

    opens com.typ.rloader.wcu to javafx.fxml;
    exports com.typ.rloader.wcu;
}