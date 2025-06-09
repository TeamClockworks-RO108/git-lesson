import javax.xml.namespace.QName;

public class Mihnea {
    private int varsta;
    public Mihnea(int varsta){
        this.varsta = varsta;
    }
    private String nume = "munteanu", prenume = "mihnea", materia_prof = "mate";
    public void display(){
        System.out.println(nume + " " + prenume + " " + materia_prof + " " + varsta);
    }
}

