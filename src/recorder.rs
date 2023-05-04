use crate::integrator::Stateful;
use bevy::prelude::*;
use rusqlite::Connection;

#[derive(Debug)]
pub struct Recorder {
    conn: Connection,
    insert_stmt: Option<String>,
}

pub fn recorder_system<T: Component + Stateful>(
    mut recorder: NonSendMut<Recorder>,
    time: Res<Time>,
    query: Query<&T>,
) {
    insert_data(&mut recorder, &query, time.elapsed_seconds() as f32);
}

pub fn initialize_recorder<T: Component + Stateful>(
    mut recorder: NonSendMut<Recorder>,
    query: Query<&T>,
) {
    create_table(&mut recorder, &query);
    insert_data(&mut recorder, &query, 0.0)
}

fn create_table<T: Component + Stateful>(recorder: &mut Recorder, query: &Query<&T>) {
    // build sql table
    let mut sql_table_defenition = String::new();
    sql_table_defenition.push_str("CREATE TABLE IF NOT EXISTS data (");
    sql_table_defenition.push_str("time REAL, ");

    let mut sql_table_insert = String::new();
    sql_table_insert.push_str("INSERT INTO data (");
    sql_table_insert.push_str("time, ");

    // add state and dstate columns
    let mut sql_params = Vec::<f32>::new();
    sql_params.push(0.0);
    for joint in query.iter() {
        let name = joint.get_name();
        println!("{}", name);
        let state_name = format!("{}_state", joint.get_name());
        let dstate_name = format!("{}_dstate", joint.get_name());
        sql_table_defenition.push_str(&format!("{} REAL, ", state_name));
        sql_table_defenition.push_str(&format!("{} REAL, ", dstate_name));
        sql_table_insert.push_str(&format!("{}, {}, ", state_name, dstate_name));
        sql_params.push(joint.get_state().into());
        sql_params.push(joint.get_dstate().into());
    }

    // remove last comma
    sql_table_defenition.pop();
    sql_table_defenition.pop();
    sql_table_defenition.push_str(")");

    sql_table_insert.pop();
    sql_table_insert.pop();
    sql_table_insert.push_str(") VALUES (");
    sql_table_insert.push_str("?, ");
    for _ in query.iter() {
        sql_table_insert.push_str("?, ?, ");
    }
    sql_table_insert.pop();
    sql_table_insert.pop();
    sql_table_insert.push_str(")");

    println!("{}", sql_table_defenition);
    println!("{}", sql_table_insert);

    // create table
    recorder
        .conn
        .execute(sql_table_defenition.as_str(), ())
        .unwrap();

    // insert data
    recorder.insert_stmt = Some(sql_table_insert);
}

fn insert_data<T: Component + Stateful>(recorder: &mut Recorder, query: &Query<&T>, time: f32) {
    let mut sql_params = Vec::<f32>::new();
    sql_params.push(time);
    for joint in query.iter() {
        sql_params.push(joint.get_state().into());
        sql_params.push(joint.get_dstate().into());
    }
    let sql_params: Vec<&dyn rusqlite::types::ToSql> = sql_params
        .iter()
        .map(|x| x as &dyn rusqlite::types::ToSql)
        .collect();
    let temp = recorder.insert_stmt.clone().unwrap();
    let mut stmt = recorder.conn.prepare(&temp).unwrap();
    stmt.execute(sql_params.as_slice()).unwrap();
}

pub fn create_recorder(world: &mut World) {
    let folder_name = "./data";
    let file_name = "dummy.db";
    let full_path = format!("{}/{}", folder_name, file_name);

    // create folder if it doesn't exist
    if !std::path::Path::new(folder_name).exists() {
        std::fs::create_dir(folder_name).unwrap();
    }

    let conn = Connection::open(full_path).unwrap();

    world.insert_non_send_resource(Recorder {
        conn: conn,
        insert_stmt: None,
    });

    println!("Recorder created")
}

fn read_data(folder_name: String, file_name: String) {
    let full_path = format!("{}/{}", folder_name, file_name);
    let conn = Connection::open(full_path).unwrap();

    let mut stmt = conn
        .prepare("SELECT * FROM sqlite_master WHERE type='table'")
        .unwrap();

    let mut tables = stmt.query([]).unwrap();
    let mut table_names = Vec::<String>::new();
    let mut table_types = Vec::<String>::new();
    while let Ok(table) = tables.next() {
        if let Some(t) = table {
            let name = t.get::<_, String>(1).unwrap();
            let type_ = t.get::<_, String>(2).unwrap();
            println!("{}: {}", name, type_);
            table_names.push(name);
            table_types.push(type_);
        } else {
            break;
        }
    }
    println!("{:?}", table_names);
    // for each table print the column names
    for table_name in table_names.iter() {
        let stmt = conn
            .prepare(format!("SELECT * FROM {}", table_name).as_str())
            .unwrap();
        let columns = stmt.column_names();
        println!("{:?}", columns);

        // // print every 30th row of the table
        // let mut rows = stmt.query([]).unwrap();
        // let mut i = 0;
        // while let Ok(row) = rows.next() {
        //     if i % 30 == 0 {
        //         if let Some(r) = row {
        //             println!("{:?}", r);
        //         } else {
        //             break;
        //         }
        //     }
        //     i += 1;
        // }
    }
}
