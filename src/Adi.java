public class Adi {
    private String prenume = "Adrian";
    private String nume = "Rosu";
    private int varsta = 12;
    private String materiaPreferata = "Fizica";

    public Adi() {}
    public Adi(int varsta) {
        this.varsta = varsta;
    }

    public void displayInfo() {
        System.out.printf("""
                %s %s
                varsta: %d
                Materia preferata: %s
                """, prenume, nume, varsta, materiaPreferata);
    }
}
