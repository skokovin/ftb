fn main() {
    if std::env::var("CARGO_CFG_TARGET_OS").unwrap() == "windows" {
        let mut res = winres::WindowsResource::new();
        res
            .set_icon("assets/icons/icon.ico")
            .set("ProductName", "Cansa Makina 2025")
            .set("FileDescription", "Pipe Bending Utility")
            .set("LegalCopyright", "Copyright © 2025 s.kokovin")
            .set("Subsystem", "windows");
        

        res.compile().unwrap();
    }
}